"""
emg_bridge.py
------------------------------------------------------------------
Turns the ESP32 raw 2-channel EMG stream ("seq,micros,quad,ham\\n" at
1000 Hz over USB serial) into the single normalized -1..+1 quad-vs-
hamstring control signal that the motor loop (virtual_torque2.py) expects.

Per-channel signal chain — all O(1), streaming, no scipy:
    raw ADC
      -> despike hard rails (isolated 0 / 4095 glitches)
      -> 1st-order high-pass (~20 Hz): removes DC offset + motion drift
      -> rectify
      -> EMA envelope (~50 ms): activation level
      -> normalize by calibrated MVC, clamp to [0, 1]

    control value = quad_norm - ham_norm      (in [-1, +1])
        +1  = full quad / extension intent
        -1  = full hamstring / flexion intent

Staleness watchdog: if no fresh sample arrives within STALE_S, get_value()
returns 0.0, so a stalled/disconnected EMG feed removes motor drive while
gravity compensation still holds the limb — it never latches a stale command.

The SAME ChannelFilter runs here and in calibrate_mvc.py, so the MVC maxima
are always measured with the identical filter used at control time.
"""

import json
import math
import time
import threading
from pathlib import Path

import serial

# -------------------- Config --------------------
PORT     = "/dev/ttyACM0"   # ESP32-S3 native USB. Classic ESP32 bridge: "/dev/ttyUSB0"
BAUD     = 921600
ADC_MAX  = 4095
FS       = 1000.0           # ESP sample rate (Hz)

HP_FC    = 20.0             # high-pass cutoff (Hz) — DC + motion-artifact removal
ENV_TAU  = 0.050           # envelope smoothing time constant (s)
STALE_S  = 0.10            # EMG older than this -> control value forced to 0
MVC_FILE = "mvc.json"
# ------------------------------------------------


class ChannelFilter:
    """Streaming DC-block high-pass -> rectify -> EMA envelope, one channel."""

    def __init__(self, fs=FS, hp_fc=HP_FC, env_tau=ENV_TAU):
        self.R = math.exp(-2.0 * math.pi * hp_fc / fs)       # high-pass pole
        self.beta = 1.0 - math.exp(-1.0 / (env_tau * fs))    # EMA coefficient
        self.x_prev = 0.0
        self.y_prev = 0.0
        self.env = 0.0
        self.raw_prev = 0.0
        self._init = False

    def process(self, raw):
        # Hold last good value through rail glitches (0 / 4095 dropouts)
        if raw <= 0 or raw >= ADC_MAX:
            raw = self.raw_prev

        # Seed state on the first real sample to avoid a startup step transient
        if not self._init:
            self.raw_prev = raw
            self.x_prev = float(raw)
            self._init = True
            return 0.0

        self.raw_prev = raw
        x = float(raw)
        # 1st-order high-pass / DC blocker:  y[n] = R*y[n-1] + x[n] - x[n-1]
        y = self.R * self.y_prev + x - self.x_prev
        self.x_prev = x
        self.y_prev = y
        # rectify + exponential-moving-average envelope
        self.env += self.beta * (abs(y) - self.env)
        return self.env


class EMGControlSource:
    """Background thread: raw ESP stream -> normalized -1..+1 control value.

    Drop-in replacement for the old serial_reader/emg_value pair. Call start(),
    read get_value() from the control loop, call stop() on shutdown.
    """

    def __init__(self, port=PORT, baud=BAUD, mvc_file=MVC_FILE):
        self.port, self.baud = port, baud
        self.q_filt = ChannelFilter()
        self.h_filt = ChannelFilter()

        mvc = load_mvc(mvc_file)
        self.mvc_quad = mvc["quad"]
        self.mvc_ham = mvc["ham"]

        self._lock = threading.Lock()
        self._value = 0.0
        self._env_q = 0.0
        self._env_h = 0.0
        self._last_update = 0.0
        self._stop = threading.Event()
        self.err = None
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2)

    def _run(self):
        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.02)
        except Exception as e:
            self.err = e
            return
        ser.reset_input_buffer()

        # Drain the connect-time backlog (the ESP samples continuously), so
        # control starts on live data, not a burst of stale queued samples.
        drain_end = time.monotonic() + 0.3
        while time.monotonic() < drain_end:
            ser.read(65536)
        ser.reset_input_buffer()

        buf = bytearray()
        try:
            while not self._stop.is_set():
                chunk = ser.read(4096)
                if not chunk:
                    continue
                buf.extend(chunk)
                while True:
                    nl = buf.find(b"\n")
                    if nl < 0:
                        break
                    line = buf[:nl]
                    del buf[:nl + 1]
                    try:
                        _, _, q_str, h_str = line.decode("ascii", "ignore").strip().split(",")
                        q = int(q_str)
                        h = int(h_str)
                    except ValueError:
                        continue
                    if not (0 <= q <= ADC_MAX and 0 <= h <= ADC_MAX):
                        continue

                    env_q = self.q_filt.process(q)
                    env_h = self.h_filt.process(h)
                    nq = min(env_q / self.mvc_quad, 1.0) if self.mvc_quad > 0 else 0.0
                    nh = min(env_h / self.mvc_ham, 1.0) if self.mvc_ham > 0 else 0.0
                    val = nq - nh

                    now = time.monotonic()
                    with self._lock:
                        self._value = val
                        self._env_q = env_q
                        self._env_h = env_h
                        self._last_update = now
        finally:
            ser.close()

    def get_value(self):
        """Latest normalized -1..+1 control value, or 0.0 if the EMG is stale."""
        now = time.monotonic()
        with self._lock:
            if now - self._last_update > STALE_S:
                return 0.0
            return self._value

    def get_envelopes(self):
        """Latest raw (un-normalized) channel envelopes — used by calibration."""
        with self._lock:
            return self._env_q, self._env_h

    def is_stale(self):
        with self._lock:
            return (time.monotonic() - self._last_update) > STALE_S


def load_mvc(path=MVC_FILE):
    if path is None:
        return {"quad": 1.0, "ham": 1.0}
    p = Path(path)
    if not p.exists():
        print(f"[emg_bridge] WARNING: {path} not found — using MVC=1.0 "
              f"(uncalibrated; control signal will be tiny). Run calibrate_mvc.py first.")
        return {"quad": 1.0, "ham": 1.0}
    with open(p) as f:
        d = json.load(f)
    return {"quad": float(d["quad"]), "ham": float(d["ham"])}


def save_mvc(quad, ham, path=MVC_FILE):
    with open(path, "w") as f:
        json.dump({"quad": quad, "ham": ham}, f, indent=2)
    print(f"[emg_bridge] saved MVC  quad={quad:.1f}  ham={ham:.1f}  -> {path}")
