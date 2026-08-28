# EMG Acquisition and Control Chain for a Powered Knee Brace

Two-channel surface EMG sampled at 1000 Hz and converted into a normalized control signal
for the motor loop of a powered knee brace. Developed with Ochsner Therapy & Wellness and
LSU Bioengineering.

## Signal path

```
2x MyoWare surface EMG (quadriceps, hamstring)
  |  ESP32-S3, hardware timer ISR at 1000 Hz
  v  "seq,micros,quad,ham\n" over USB CDC at 921600 baud
emg_bridge.py
  raw ADC
  -> despike hard rails (isolated 0 / 4095 readings)
  -> first-order high-pass at about 20 Hz, removing DC offset and motion drift
  -> rectify
  -> EMA envelope over about 50 ms
  -> normalize against calibrated MVC, clamp to [0, 1]
  |
  v  control = quad_norm - ham_norm, range [-1, +1]
virtual_torque2.py  ->  motor loop with gravity compensation
```

A control value of +1 corresponds to full extension intent and -1 to full flexion intent.

## Timing

Sampling is driven by a hardware timer interrupt on the ESP32-S3, so the rate does not
depend on host scheduling. Each line carries a monotonic sequence index, and gaps in that
index indicate dropped samples. `micros()` is transmitted for diagnostics only, since it can
jump by whole seconds on some ESP32 cores.

## Watchdog

If no fresh sample arrives within the timeout, `get_value()` returns 0.0. Motor drive is
removed while gravity compensation continues to hold the limb, so a stalled or disconnected
feed does not leave a stale command in effect.

## Calibration

`calibrate_mvc.py` and `emg_bridge.py` use the same `ChannelFilter`, so MVC maxima are
measured through the same filter later used to normalize against them.

## Files

| File | Purpose |
|---|---|
| `firmware/brace_emg_esp/` | ESP32-S3 dual-channel sampler |
| `firmware/norm_emg_smoothed/` | On-device normalization variant |
| `emg_bridge.py` | Serial reader, filter chain, control signal, watchdog |
| `calibrate_mvc.py` | Maximum voluntary contraction calibration |
| `brace_emg_logger.py` | Raw capture logging |
| `virtual_torque2.py` | Motor control loop with gravity compensation |

## Hardware

Power each MyoWare board at 3.3 V. The RAW output is centered at Vcc/2 and swings toward the
rails, so at 5 V it exceeds the ESP32 ADC input range.

## Stack

C++ (Arduino, ESP32-S3), Python, pyserial. The filter chain is streaming and does not
require SciPy.
