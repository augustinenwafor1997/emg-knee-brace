import sys
import time
import serial
import math
import threading
import emg_bridge as eb
from cubemars_can2 import CubeMarsServoCAN
from opensourceleg.utilities.softrealtimeloop import SoftRealtimeLoop

FREQUENCY   = 200       # Hz
DT          = 1 / FREQUENCY
MOTOR_ID    = 104

MAX_TAU_CMD = 2 #N*m  ADJUST THROUGHOUT TESTING

# ─────────────────────────────────────────────────────────
#  VIRTUAL LEG PARAMETERS
# ─────────────────────────────────────────────────────────
'''
DV:
The virtual leg parameters serve as the basis of a "simulated tibia" that will dictate the movement of the brace
We are computing what speed this virtual tibia WOULD move based on torque values from EMG command and gravity
The virtual speed is what commands the motor
Adjust parameters based on leg measurements, estimate COM at 40% of tib length (i.e. 0.5 m tibia, l_c = 0.4*0.5 = 0.2 meters)
ALL METRIC UNITS
'''
INERTIA     = 0.25      # Virtual inertia of tibia (kg·m²) — tune
B_VISC      = 1.5       # Virtual viscous damping (N·m·s/rad) — prevents runaway
VEL_MAX     = 0.2       # Hard ceiling on |omega_cmd| (rad/s)   ADJUST THROUGHOUT TESTING!!!!!!!
ALPHA_MAX   = 4.0       # Max angular acceleration (rad/s²) — limits jerk felt by wearer

# Gravity model
MGLC        = 16.0      # Effective m·g·l_c (N·m) — tune
THETA_DOWN  = 0.0       # Angle where tibia hangs straight down (deg)
                        # Replace with live IMU reading when available

# ─────────────────────────────────────────────────────────
#  JOINT ANGLE BOUNDARIES
# ─────────────────────────────────────────────────────────
ANGLE_MAX_DEG   =   0.0     # Full extension hard stop
ANGLE_MIN_DEG   = -65.0     # Full flexion hard stop

# Soft-limit zone: deceleration begins this many degrees before the hard stop.
# Inside this zone the integrator is zeroed if it would push further into the limit.
SOFT_MARGIN_DEG =   5.0

# EMG input is provided by emg_bridge.EMGControlSource, which reads the ESP32
# raw 2-channel stream, computes the normalized quad-vs-hamstring control value
# (-1..+1), and forces 0.0 whenever the feed goes stale (built-in watchdog).
# ─────────────────────────────────────────────────────────
#  BOUNDARY HELPERS
# ─────────────────────────────────────────────────────────

def boundary_alpha_scale(theta_deg: float, omega: float) -> float:
    """
    Returns a [0, 1] scalar applied to the computed acceleration (alpha)
    before it is integrated into omega_cmd.

    Within SOFT_MARGIN_DEG of a hard stop, only acceleration that would
    drive the joint *further* into the limit is suppressed.  Acceleration
    away from the limit is always permitted, so the joint can always recover.

    Args:
        theta_deg: Current output-shaft position in degrees.
        omega:     Current virtual velocity (rad/s).
                   Positive → extension (toward ANGLE_MAX_DEG, zero).
                   Negative → flexion (toward ANGLE_MIN_DEG, more negative).
    """
    scale = 1.0

    # Extension boundary: theta approaching ANGLE_MAX_DEG (0°) from below.
    # omega > 0 means moving toward extension (positive direction).
    if omega > 0:
        margin = ANGLE_MAX_DEG - theta_deg          # positive while inside range
        if margin <= 0.0:
            return 0.0
        if margin < SOFT_MARGIN_DEG:
            scale = min(scale, margin / SOFT_MARGIN_DEG)

    # Flexion boundary: theta approaching ANGLE_MIN_DEG (-65°) from above.
    # omega < 0 means moving toward flexion (negative direction).
    if omega < 0:
        margin = theta_deg - ANGLE_MIN_DEG          # positive while inside range
        if margin <= 0.0:
            return 0.0
        if margin < SOFT_MARGIN_DEG:
            scale = min(scale, margin / SOFT_MARGIN_DEG)

    return max(0.0, scale)


def integrator_anti_windup(theta_deg: float, omega: float) -> float:
    """
    If the joint is already outside (or exactly at) a hard stop, zero the
    virtual velocity unless it is directed back into the safe range.

    This prevents the integrator from accumulating a large velocity while
    the motor is physically blocked at a mechanical stop.
    """
    if theta_deg >= ANGLE_MAX_DEG and omega > 0:
        # Past extension limit, still pushing extension (positive) — kill it.
        return 0.0
    if theta_deg <= ANGLE_MIN_DEG and omega < 0:
        # Past flexion limit, still pushing flexion (negative) — kill it.
        return 0.0
    return omega


def boundary_velocity_limit(theta_deg: float, omega_cmd: float) -> float:
    """
    Directly caps omega_cmd based on proximity to each boundary, independent
    of the virtual dynamics.  Only suppresses velocity in the direction that
    would worsen a limit violation — recovery direction is always unrestricted.

    This acts as a second, direct enforcement layer on top of boundary_alpha_scale,
    ensuring a high incoming velocity cannot carry the joint through the soft zone
    faster than the alpha-scaling can shed it.

    Args:
        theta_deg: Current output-shaft position in degrees.
        omega_cmd: Current velocity command (rad/s).
                   Positive → extension (toward ANGLE_MAX_DEG, zero).
                   Negative → flexion (toward ANGLE_MIN_DEG, more negative).
    """
    # Approaching or past extension limit — cap positive velocity only
    if omega_cmd > 0:
        margin = ANGLE_MAX_DEG - theta_deg
        if margin <= 0.0:
            return 0.0
        if margin < SOFT_MARGIN_DEG:
            omega_cmd = min(omega_cmd, VEL_MAX * (margin / SOFT_MARGIN_DEG))

    # Approaching or past flexion limit — cap negative velocity only
    if omega_cmd < 0:
        margin = theta_deg - ANGLE_MIN_DEG
        if margin <= 0.0:
            return 0.0
        if margin < SOFT_MARGIN_DEG:
            omega_cmd = max(omega_cmd, -VEL_MAX * (margin / SOFT_MARGIN_DEG))

    return omega_cmd


# ─────────────────────────────────────────────────────────
#  EMG / DESIRED TORQUE  (replace with your real pipeline)
# ─────────────────────────────────────────────────────────

def get_desired_torque(emg_value: float) -> float:
    """
    Read EMG signals and return desired joint torque (N·m).
    emg_value is normalized value from -1 to 1 representing effort in either direction
    Negative value, negative torque → flexion.
    Positive value, positive torque → extension.
    """
    tau_cmd = emg_value * MAX_TAU_CMD
    return tau_cmd

# ─────────────────────────────────────────────────────────
#  WATCHDOG
# ─────────────────────────────────────────────────────────

# Maximum time (seconds) allowed without a valid feedback message before
# the controller commands zero velocity and raises an error.
FEEDBACK_TIMEOUT_S = 0.5


# ─────────────────────────────────────────────────────────
#  MAIN CONTROL LOOP
# ─────────────────────────────────────────────────────────

def velocity_control():
    motor = CubeMarsServoCAN(channel="can0", bustype="socketcan")
    clock = SoftRealtimeLoop(dt=DT)

    omega_cmd        = 0.0      # Virtual velocity state (rad/s)
    theta_deg        = None      # Last known position — updated on every valid message
    omega_meas       = 0.0      # Last known measured speed
    last_feedback_t  = None     # Wall-clock time of last valid feedback message

    emg_source = eb.EMGControlSource()
    emg_source.start()
    time.sleep(0.3)
    if emg_source.err is not None:
        raise RuntimeError(f"EMG source failed to open {eb.PORT}: {emg_source.err}")

    print("Starting virtual-dynamics knee controller.")
    print(f"  Angle limits    : {ANGLE_MIN_DEG}° to {ANGLE_MAX_DEG}°")
    print(f"  Soft margin     : {SOFT_MARGIN_DEG}°")
    print(f"  VEL_MAX         : ±{VEL_MAX} rad/s")
    print(f"  ALPHA_MAX       : ±{ALPHA_MAX} rad/s²")
    print(f"  Feedback timeout: {FEEDBACK_TIMEOUT_S} s")
    print(f"  EMG source      : {eb.PORT}  (MVC quad={emg_source.mvc_quad:.0f} "
          f"ham={emg_source.mvc_ham:.0f}, stale>{eb.STALE_S*1000:.0f}ms -> 0)")
    print()

    with motor:
        try:
            for t in clock:

                # ── 0.5. Read latest EMG value (0.0 if the feed is stale) ──
                current_emg = emg_source.get_value()

                # ── 1. Read motor feedback ────────────────────────────────────
                msg   = motor.recv_feedback(timeout=0.001)
                state = motor.decode_servo_feedback(msg)

                if state:
                    theta_deg       = state["position_deg"]
                    omega_meas      = state.get("speed_erpm", 0.0)
                    last_feedback_t = time.monotonic()
                else:
                    # ── Watchdog check ────────────────────────────────────────
                    if last_feedback_t is None:
                        # No feedback ever received — motor may not be online yet.
                        # Hold zero and wait; do not start integrating blind.
                        motor.set_output_speed(MOTOR_ID, 0.0)
                        continue

                    elapsed = time.monotonic() - last_feedback_t
                    if elapsed > FEEDBACK_TIMEOUT_S:
                        motor.set_output_speed(MOTOR_ID, 0.0)
                        raise RuntimeError(
                            f"[WATCHDOG] No feedback for {elapsed:.2f}s "
                            f"(limit {FEEDBACK_TIMEOUT_S}s). "
                            f"Last known position: {theta_deg:.2f}°. "
                            f"Motor commanded to zero and stopped."
                        )
                #Avoid firing when CAN hasn't started sending angles
                if theta_deg is None:
                    motor.set_output_speed(MOTOR_ID, 0.0)
                    continue

                    # Feedback gap is within tolerance — hold last command,
                    # continue with stale-but-real theta_deg for boundary logic.

                # ── 2. Anti-windup: zero integrator if already past a hard stop ─
                omega_cmd = integrator_anti_windup(theta_deg, omega_cmd)

                # ── 3. Gravity compensation torque ────────────────────────────
                # sin(-theta_deg * π/180) matches original sign convention
                theta_rad = math.radians(theta_deg - THETA_DOWN)
                tau_g = MGLC * math.sin(-theta_rad)              #NOTE: ADD IMU TO MEASURE THETA_DOWN. USE THIS TO MAKE TAU_G MORE ACCURATE

                # ── 4. EMG-derived desired torque ─────────────────────────────
                tau_cmd = get_desired_torque(current_emg)

                # ── 5. Net torque → acceleration ──────────────────────────────
                tau_net = tau_cmd + tau_g - B_VISC * omega_cmd
                alpha   = tau_net / INERTIA

                # ── 6. Clamp acceleration (limits jerk on the wearer) ─────────
                alpha = max(-ALPHA_MAX, min(ALPHA_MAX, alpha))

                # ── 7. Soft-limit: scale alpha near boundaries ────────────────
                alpha *= boundary_alpha_scale(theta_deg, omega_cmd)

                # ── 8. Euler integrate ────────────────────────────────────────
                omega_cmd += alpha * DT

                # ── 9. Final hard velocity ceiling (last-resort safety clamp) ─
                omega_cmd = max(-VEL_MAX, min(VEL_MAX, omega_cmd))

                # ── 10. Proximity velocity cap (direct, bypasses dynamics) ────
                omega_cmd = boundary_velocity_limit(theta_deg, omega_cmd)

                # ── 11. Send command ──────────────────────────────────────────
                motor.set_output_speed(MOTOR_ID, omega_cmd)

                # ── 12. Periodic telemetry ────────────────────────────────────
                if int(t / DT) % 50 == 0:
                    near_ext  = theta_deg >= (ANGLE_MAX_DEG - SOFT_MARGIN_DEG)
                    near_flex = theta_deg <= (ANGLE_MIN_DEG + SOFT_MARGIN_DEG)
                    flag      = " ⚠ NEAR LIMIT" if (near_ext or near_flex) else ""
                    if emg_source.is_stale():
                        flag += " ⚠ EMG STALE"

                    print(
                        f"t={t:6.2f}s | "
                        f"emg={current_emg:+.3f} | "
                        f"θ={theta_deg:7.2f}° | "
                        f"τg={tau_g:+6.2f} | "
                        f"τcmd={tau_cmd:+6.2f} | "
                        f"α={alpha:+6.3f} | "
                        f"ω={omega_cmd:+6.3f} rad/s"
                        f"{flag}"
                    )
        except KeyboardInterrupt:
            print("\n[CTRL+C] Shutting down...")

        finally:
            motor.set_output_speed(MOTOR_ID, 0.0)
            time.sleep(0.1)
            motor.set_output_speed(MOTOR_ID, 0.0)
            emg_source.stop()

def dry_run_monitor():
    """Run the full EMG path and print the live control value — motor NEVER commanded.

    Use this to validate calibration and 'signal feel' before anything moves:
    watch that rest sits near 0, a quad contraction drives toward +1, a hamstring
    contraction toward -1, and that unplugging the ESP forces the value to 0 (STALE).
    """
    def bar(v, width=41):
        v = max(-1.0, min(1.0, v))
        mid = width // 2
        pos = int(round((v + 1.0) / 2.0 * (width - 1)))
        cells = ["·"] * width
        cells[mid] = "|"
        cells[pos] = "#"
        return "".join(cells)

    emg_source = eb.EMGControlSource()
    emg_source.start()
    time.sleep(0.3)
    if emg_source.err is not None:
        print(f"EMG source failed to open {eb.PORT}: {emg_source.err}")
        return

    print("DRY RUN — reading EMG only, motor is NOT commanded. Ctrl+C to stop.")
    print(f"  MVC quad={emg_source.mvc_quad:.0f} ham={emg_source.mvc_ham:.0f}  "
          f"stale>{eb.STALE_S*1000:.0f}ms -> 0")
    print("  flexion -1 <" + " " * 17 + "0" + " " * 17 + "> +1 extension\n")
    try:
        while True:
            val = emg_source.get_value()
            eq, eh = emg_source.get_envelopes()
            stale = " STALE" if emg_source.is_stale() else "      "
            print(f"  emg={val:+.3f} [{bar(val)}] q={eq:6.1f} h={eh:6.1f}{stale}",
                  end="\r", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        emg_source.stop()


if __name__ == "__main__":
    if "--dry-run" in sys.argv:
        dry_run_monitor()
    else:
        velocity_control()
