"""
Brace EMG logger — Pi side
------------------------------------------------------------------
Receives the ESP32-S3 dual-MyoWare stream ("seq,micros,quad,ham\\n") over
USB serial, records until you stop, then saves and runs an audit so you
can confirm the true 1000 Hz rate and zero dropped samples. The time base
is the sample index (seq), which is immune to the ESP micros() glitches.

Serial reading runs in its own thread, so the optional live plot can
NEVER slow the read loop down and cause dropped samples.

Outputs (in ./data):
    <name>.csv        ArduinoMicros, TimeFromStart, EMG_Quad, EMG_Ham   (combined)
    <name>_quad.csv   ArduinoMicros, TimeFromStart, EMG_Value           (drop-in)
    <name>_ham.csv    ArduinoMicros, TimeFromStart, EMG_Value           (drop-in)

The per-channel files use the exact schema that bandpass_filter.py and
active_fft.py already expect — point those scripts at either one.

Usage:
    python brace_emg_logger.py [name]            # console status only
    python brace_emg_logger.py [name] --plot     # live scrolling plot too
    (name optional; defaults to a timestamp.
     Console mode: Ctrl+C to stop & save.
     Plot mode:    close the window (or Ctrl+C) to stop & save.)

Plot mode needs matplotlib and a desktop/display:
    pip3 install matplotlib --break-system-packages
"""

import sys
import gc
import time
import threading
import collections
import serial
import numpy as np
import pandas as pd
from pathlib import Path

# -------------------- Config --------------------
PORT = "/dev/ttyACM0"     # ESP32-S3 native USB. Classic ESP32 bridge: "/dev/ttyUSB0"
BAUD = 921600             # must match firmware SERIAL_BAUD (ignored by native USB)
WARMUP_TRIM_S = 1.0       # discard this many seconds at the start (sensor/ADC settling)
ADC_MAX = 4095            # 12-bit
PLOT_WINDOW_S = 3.0       # seconds of history shown in the live plot
PLOT_REFRESH_MS = 50      # plot redraw interval (~20 FPS); reading is unaffected
# ------------------------------------------------


class SerialReader(threading.Thread):
    """Tight serial read loop in its own thread.

    Keeps the full record in `samples` and a short rolling window
    (`plot_*` deques) for the live view. Because this runs independently
    of any drawing, plotting cannot introduce sample drops.
    """

    def __init__(self, port, baud):
        super().__init__(daemon=True)
        self.port, self.baud = port, baud
        self.samples = []                 # full record: list of (seq, micros, quad, ham)
        self.stop_event = threading.Event()
        self.err = None                   # exception from the thread, if any
        self.rate = 0.0                   # instantaneous Hz (updated ~1/s)

        maxlen = int(1000 * PLOT_WINDOW_S)
        self.plot_t = collections.deque(maxlen=maxlen)
        self.plot_q = collections.deque(maxlen=maxlen)
        self.plot_h = collections.deque(maxlen=maxlen)

    def run(self):
        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.05)
        except Exception as e:            # connection failed — hand it back to main
            self.err = e
            return

        ser.reset_input_buffer()

        # The ESP samples continuously whether or not a host is reading, so on
        # connect there is usually a burst of stale/queued data (its Seq counter
        # is already high). Drain ~0.3 s and flush, so we start on live samples.
        drain_end = time.monotonic() + 0.3
        while time.monotonic() < drain_end:
            ser.read(65536)
        ser.reset_input_buffer()

        buf = bytearray()
        first_line_skipped = False
        t_report = time.monotonic()
        last_n = 0

        # Cyclic GC is the enemy here: as the sample list grows, a periodic GC
        # pass stalls THIS thread for ~100 ms, the serial buffer backs up, and
        # the ESP drops samples during the stall. Disable it for the capture —
        # refcounting still frees everything and the record is bounded.
        gc.disable()
        try:
            while not self.stop_event.is_set():
                chunk = ser.read(65536)          # bulk read drains the OS buffer in one shot
                if chunk:
                    buf.extend(chunk)
                    while True:                  # pull out every complete line
                        nl = buf.find(b"\n")
                        if nl < 0:
                            break
                        line = buf[:nl]
                        del buf[:nl + 1]
                        try:
                            s_str, t_str, q_str, h_str = line.decode("ascii", "ignore").strip().split(",")
                            seq, t, q, h = int(s_str), int(t_str), int(q_str), int(h_str)
                        except ValueError:
                            continue  # partial/corrupt line — recovers on next newline

                        # First parsed line may be a mid-stream fragment; drop it.
                        if not first_line_skipped:
                            first_line_skipped = True
                            continue

                        if not (0 <= q <= ADC_MAX and 0 <= h <= ADC_MAX):
                            continue

                        self.samples.append((seq, t, q, h))
                        self.plot_t.append(seq)          # seq is the clean time base (1 unit = 1 ms)
                        self.plot_q.append(q)
                        self.plot_h.append(h)

                now = time.monotonic()
                if now - t_report >= 1.0:
                    self.rate = (len(self.samples) - last_n) / (now - t_report)
                    t_report, last_n = now, len(self.samples)
        finally:
            gc.enable()
            ser.close()


def run_console(reader):
    """Start the reader and print a one-line status roughly once a second."""
    reader.start()
    time.sleep(0.3)
    if reader.err:
        raise reader.err

    print("Connected. Recording... press Ctrl+C to stop and save.\n")
    try:
        while True:
            time.sleep(1.0)
            if reader.err:
                raise reader.err
            if reader.samples:
                _, _, q, h = reader.samples[-1]
                print(f"  {len(reader.samples):>7} samples | ~{reader.rate:6.1f} Hz | "
                      f"quad={q:4d} ham={h:4d}")
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        reader.stop_event.set()
        reader.join(timeout=2)


def run_plot(reader):
    """Start the reader and show a live scrolling plot of both channels.

    The reader thread does all the serial work; this only visualizes the
    rolling window. Closing the window (or Ctrl+C) stops the recording.
    """
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    reader.start()
    time.sleep(0.3)
    if reader.err:
        raise reader.err

    fig, (ax_q, ax_h) = plt.subplots(2, 1, sharex=True, figsize=(9, 5))
    fig.suptitle("Brace EMG — live  (close window or Ctrl+C to stop & save)")
    (line_q,) = ax_q.plot([], [], lw=0.8, color="tab:blue")
    (line_h,) = ax_h.plot([], [], lw=0.8, color="tab:red")
    for ax, lbl in ((ax_q, "QUAD"), (ax_h, "HAM")):
        ax.set_ylim(0, ADC_MAX)
        ax.set_xlim(0, PLOT_WINDOW_S)
        ax.set_ylabel(lbl)
        ax.grid(alpha=0.3)
    ax_h.set_xlabel("seconds (rolling)")

    def update(_frame):
        if not reader.plot_t:
            return line_q, line_h
        t = np.fromiter(reader.plot_t, dtype=np.int64)
        ts = (t - t[0]) / 1000.0          # seq units are ms → seconds
        line_q.set_data(ts, np.fromiter(reader.plot_q, dtype=np.int32))
        line_h.set_data(ts, np.fromiter(reader.plot_h, dtype=np.int32))
        ax_q.set_title(f"{len(reader.samples)} samples   ~{reader.rate:.0f} Hz",
                       fontsize=9, loc="right")
        return line_q, line_h

    anim = FuncAnimation(fig, update, interval=PLOT_REFRESH_MS,
                         blit=False, cache_frame_data=False)
    fig.canvas.mpl_connect("close_event", lambda _e: reader.stop_event.set())

    try:
        plt.show()          # blocks until the window is closed
    except KeyboardInterrupt:
        pass
    finally:
        reader.stop_event.set()
        reader.join(timeout=2)
        print("\nStopped.")
    _ = anim               # keep a reference so it isn't garbage-collected


def save_and_audit(samples, name):
    if len(samples) < 100:
        print(f"Only {len(samples)} samples — nothing worth saving.")
        return

    df = pd.DataFrame(samples, columns=["Seq", "RawMicros", "EMG_Quad", "EMG_Ham"])

    # Clean time base: sample index x 1 ms. Immune to micros() glitches, and
    # gaps in Seq are the ONLY true measure of dropped samples.
    df["Seq"] = df["Seq"] - df["Seq"].iloc[0]
    df["ArduinoMicros"] = df["Seq"] * 1000          # microseconds, glitch-free
    df["TimeFromStart"] = df["Seq"] / 1000.0        # seconds

    # Trim warm-up transient and re-zero
    if WARMUP_TRIM_S > 0:
        df = df[df["TimeFromStart"] >= WARMUP_TRIM_S].reset_index(drop=True)
        df["Seq"] = df["Seq"] - df["Seq"].iloc[0]
        df["ArduinoMicros"] = df["Seq"] * 1000
        df["TimeFromStart"] = round(df["Seq"] / 1000.0, 6)

    # ---- Audit (truth = Seq) ----
    dseq = np.diff(df["Seq"].to_numpy(np.int64))
    dropped = int(np.sum(dseq[dseq > 1] - 1))      # missing sample indices
    drop_events = int(np.sum(dseq > 1))
    dur = df["TimeFromStart"].iloc[-1]
    rate = len(df) / dur if dur > 0 else float("nan")

    # ---- Diagnostic (micros glitches — should NOT affect the numbers above) ----
    draw_ms = np.diff(df["RawMicros"].to_numpy(np.int64)) / 1000.0
    micros_glitches = int(np.sum(np.abs(draw_ms - 1.0) > 0.5))

    print("\n----- audit (Seq = truth) -----")
    print(f"  samples        : {len(df)}")
    print(f"  duration       : {dur:.2f} s")
    print(f"  rate           : {rate:.2f} Hz   (target 1000)")
    print(f"  dropped samples: {dropped}  (in {drop_events} gaps)")
    print(f"  QUAD  min/mean/max : {df['EMG_Quad'].min()}/"
          f"{df['EMG_Quad'].mean():.0f}/{df['EMG_Quad'].max()}")
    print(f"  HAM   min/mean/max : {df['EMG_Ham'].min()}/"
          f"{df['EMG_Ham'].mean():.0f}/{df['EMG_Ham'].max()}")

    # ---- Signal quality ----
    q = df["EMG_Quad"].to_numpy()
    h = df["EMG_Ham"].to_numpy()
    q_clip = 100.0 * ((q == 0) | (q == ADC_MAX)).mean()
    h_clip = 100.0 * ((h == 0) | (h == ADC_MAX)).mean()
    cm = float(np.corrcoef(q, h)[0, 1]) if len(q) > 1 else 0.0
    print(f"  clipping       : quad {q_clip:4.1f}%   ham {h_clip:4.1f}%   (want < ~1%)")
    print(f"  common-mode r  : {cm:+.2f}   (near +1 => channels share a signal/artifact)")
    print(f"  [diag] micros() glitches: {micros_glitches}  "
          f"(ignored - Seq is authoritative)")
    if dropped == 0 and abs(rate - 1000) < 2:
        print("  -> clean 1000 Hz, no dropped samples.")
    print("-------------------------------\n")

    # ---- Save ----
    data_dir = Path("data")
    data_dir.mkdir(exist_ok=True)

    combined = df[["ArduinoMicros", "TimeFromStart", "EMG_Quad", "EMG_Ham"]]
    combined.to_csv(data_dir / f"{name}.csv", index=False)

    for ch, col in (("quad", "EMG_Quad"), ("ham", "EMG_Ham")):
        one = df[["ArduinoMicros", "TimeFromStart", col]].rename(columns={col: "EMG_Value"})
        one.to_csv(data_dir / f"{name}_{ch}.csv", index=False)

    print(f"Saved to {data_dir}/{name}.csv (+ _quad.csv, _ham.csv)")


if __name__ == "__main__":
    argv = sys.argv[1:]
    do_plot = "--plot" in argv
    argv = [a for a in argv if a != "--plot"]

    name = argv[0] if argv else time.strftime("emg_%Y%m%d_%H%M%S")
    name = name.replace(".csv", "").strip()

    print(f"Connecting to {PORT} @ {BAUD} ...")
    reader = SerialReader(PORT, BAUD)

    try:
        (run_plot if do_plot else run_console)(reader)
    except serial.SerialException as e:
        print(f"Could not open {PORT}: {e}")
        print("Check the cable, run `ls /dev/ttyACM*`, and set PORT at the top of this file.")
    except ImportError:
        print("Plot mode needs matplotlib:  pip3 install matplotlib --break-system-packages")
    except Exception as e:
        print(f"Stopped on error: {e}")
    finally:
        reader.stop_event.set()

    save_and_audit(reader.samples, name)
