"""
calibrate_mvc.py
------------------------------------------------------------------
Measures each muscle's maximum-voluntary-contraction (MVC) envelope and
writes mvc.json, which emg_bridge / virtual_torque2 load to normalize the
control signal to -1..+1.

It runs the SAME filter chain as the live controller (via EMGControlSource),
so the maxima are valid for control. Uses the 90th percentile of the envelope
during each contraction (robust to single-sample spikes).

Run on the Pi with the ESP plugged in:
    python3 calibrate_mvc.py

Re-run per participant, or whenever electrode placement changes.
"""

import time
import numpy as np
import emg_bridge as eb


def record_contraction(src, which, seconds=3.0, countdown=3):
    for n in range(countdown, 0, -1):
        print(f"  {which.upper()} contraction in {n}...", end="\r", flush=True)
        time.sleep(1.0)
    print(f"  >>> CONTRACT {which.upper()} AS HARD AS YOU CAN — {seconds:.0f} s      ")
    vals = []
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        eq, eh = src.get_envelopes()
        vals.append(eq if which == "quad" else eh)
        time.sleep(0.005)
    print("  relax.\n")
    time.sleep(1.0)
    return np.array(vals)


def main():
    # mvc_file=None -> MVC 1.0, so get_envelopes() returns raw envelope magnitude
    src = eb.EMGControlSource(mvc_file=None)
    src.start()
    time.sleep(0.5)
    if src.err:
        print(f"Could not open {eb.PORT}: {src.err}")
        print("Check the cable and PORT at the top of emg_bridge.py.")
        return

    print("\n=== MVC calibration ===")
    print("Sit relaxed. You'll do one maximal contraction per muscle.\n")
    time.sleep(1.5)

    q = record_contraction(src, "quad")
    h = record_contraction(src, "ham")
    src.stop()

    if q.size == 0 or h.size == 0:
        print("No envelope data captured — check the EMG stream and retry.")
        return

    mvc_quad = float(np.percentile(q, 90))
    mvc_ham = float(np.percentile(h, 90))
    print(f"quad: peak={q.max():.1f}  p90={mvc_quad:.1f}")
    print(f"ham : peak={h.max():.1f}  p90={mvc_ham:.1f}")

    if mvc_quad <= 0 or mvc_ham <= 0:
        print("Got a zero/near-zero MVC — check electrodes and wiring, then retry.")
        return

    eb.save_mvc(mvc_quad, mvc_ham)
    print("Done. The controller will now normalize against these maxima.")


if __name__ == "__main__":
    main()
