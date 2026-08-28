# EMG Acquisition and Control Chain for a Powered Knee Brace

Two-channel surface EMG at a hardware-timed **1000 Hz**, turned into a single normalized
intent signal that drives the motor loop of a powered knee brace.

Developed with Ochsner Therapy & Wellness and LSU Bioengineering.

## Signal path

```
2x MyoWare surface EMG (quadriceps, hamstring)
   |  ESP32-S3, hardware timer ISR at exactly 1000 Hz
   v  "seq,micros,quad,ham\n" over USB CDC at 921600 baud
emg_bridge.py
   raw ADC
   -> despike hard rails (isolated 0 / 4095 glitches)
   -> 1st-order high-pass (~20 Hz)    removes DC offset and motion drift
   -> rectify
   -> EMA envelope (~50 ms)           activation level
   -> normalize by calibrated MVC, clamp to [0, 1]
   |
   v  control = quad_norm - ham_norm, in [-1, +1]
virtual_torque2.py  ->  motor loop with gravity compensation
```

`+1` is full extension intent, `-1` full flexion intent.

## Two decisions worth calling out

**Timing is owned by the microcontroller, not the host.** The ISR fires on a hardware timer,
so the sample rate is exact regardless of what Linux is doing. Each line carries a monotonic
sequence index; gaps in `seq` are the only truthful measure of dropped samples.
`micros()` is transmitted for diagnostics only, because it can jump by whole seconds on some
ESP32 cores.

**The system fails safe.** A staleness watchdog returns a control value of `0.0` if no fresh
sample arrives within the timeout. A stalled or unplugged EMG feed therefore removes motor
drive while gravity compensation still holds the limb - it can never latch a stale command
and keep driving.

The same `ChannelFilter` runs in `calibrate_mvc.py` and at control time, so MVC maxima are
always measured through the identical filter that will be used to normalize against them.

## Contents

| File | Purpose |
|---|---|
| `firmware/brace_emg_esp/` | ESP32-S3 dual-channel sampler, timer ISR |
| `firmware/norm_emg_smoothed/` | On-device normalization variant |
| `emg_bridge.py` | Serial reader, filter chain, control signal, watchdog |
| `calibrate_mvc.py` | Maximum voluntary contraction calibration |
| `brace_emg_logger.py` | Raw capture logging |
| `virtual_torque2.py` | Motor control loop with gravity compensation |

## Hardware note

**Power each MyoWare at 3.3 V, not 5 V.** The RAW output is centered at Vcc/2 and swings
toward the rails; at 5 V it exceeds the ESP32's ADC input range and can damage the pin.

## Stack

C++ (Arduino/ESP32-S3), Python, pyserial. No SciPy - the filter chain is O(1) streaming.
