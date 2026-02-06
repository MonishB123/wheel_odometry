# -----------------------------
# Imports
# -----------------------------
import math
import threading
import time

from USART import control_speed, receive_data, send_upload_command, ser, set_motor_parameter

# -----------------------------
# Motor direction fix (your robot)
# -----------------------------
# M2 and M4 inverted
MOTOR_DIR = [1, -1, 1, -1]

# -----------------------------
# Calibration from your measurement
# -----------------------------
# You measured: 800 ticks per wheel revolution = 0.147 m travel
TICKS_PER_REV = 800
DIST_PER_REV_M = 0.147

METERS_PER_TICK = DIST_PER_REV_M / TICKS_PER_REV      # 0.000271875 m/tick
TICKS_PER_METER = TICKS_PER_REV / DIST_PER_REV_M      # ~3678.1609 ticks/m

# -----------------------------
# Robot geometry
# -----------------------------
TRACK_WIDTH_M = 0.14  # 14 cm track width (wheel-to-wheel)

# -----------------------------
# Pose (x right, y forward, theta CCW)
# -----------------------------
x_m = 0.0
y_m = 0.0
theta_rad = 0.0

def wrap_pi(a):
    while a >= math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a

def pose_str():
    return f"x={x_m: .3f} m, y={y_m: .3f} m, theta={math.degrees(theta_rad): .1f}°"

# -----------------------------
# Latest total encoder ticks from $MAll
# -----------------------------
latest_ticks = [0, 0, 0, 0]
ticks_lock = threading.Lock()

def parse_mall(msg: str):
    """
    Input:  "$MAll:123,456,789,101#"
    Output: [t1,t2,t3,t4] as ints, direction-corrected with MOTOR_DIR
    """
    msg = msg.strip()
    if not (msg.startswith("$MAll:") and msg.endswith("#")):
        return None

    payload = msg[len("$MAll:"):-1]
    parts = payload.split(",")
    if len(parts) != 4:
        return None

    t = [int(p) for p in parts]
    t = [t[i] * MOTOR_DIR[i] for i in range(4)]
    return t

# -----------------------------
# Threads: receiver + odometry + printer
# -----------------------------
running = False

def _get_ticks():
    with ticks_lock:
        return latest_ticks[:]  # copy

def rx_loop():
    while running:
        msg = receive_data()
        if msg:
            t = parse_mall(msg)
            if t is not None:
                with ticks_lock:
                    latest_ticks[:] = t
        time.sleep(0.001)

def integrate_pose_from_tick_delta(dL_ticks: float, dR_ticks: float):
    global x_m, y_m, theta_rad

    dL_m = dL_ticks * METERS_PER_TICK
    dR_m = dR_ticks * METERS_PER_TICK

    d = 0.5 * (dL_m + dR_m)
    dtheta = (dR_m - dL_m) / TRACK_WIDTH_M

    th_mid = theta_rad + 0.5 * dtheta
    x_m += d * math.sin(th_mid)
    y_m += d * math.cos(th_mid)
    theta_rad = wrap_pi(theta_rad + dtheta)

def odom_loop(hz=60.0):
    dt_target = 1.0 / hz
    prev = _get_ticks()

    while running:
        t0 = time.time()
        now = _get_ticks()

        dL_ticks = 0.5 * ((now[0] - prev[0]) + (now[3] - prev[3]))
        dR_ticks = 0.5 * ((now[1] - prev[1]) + (now[2] - prev[2]))

        integrate_pose_from_tick_delta(dL_ticks, dR_ticks)

        prev = now
        elapsed = time.time() - t0
        sleep_t = dt_target - elapsed
        if sleep_t > 0:
            time.sleep(sleep_t)

def print_loop(hz=10.0):
    dt_target = 1.0 / hz
    while running:
        t = _get_ticks()
        s = f"ticks: M1={t[0]} M2={t[1]} M3={t[2]} M4={t[3]} | {pose_str()}"
        print("\r" + s + " " * 10, end="", flush=True)
        time.sleep(dt_target)

# -----------------------------
# Motor commands
# -----------------------------
def drive_raw(m1, m2, m3, m4):
    m = [m1, m2, m3, m4]
    m = [m[i] * MOTOR_DIR[i] for i in range(4)]
    control_speed(m[0], m[1], m[2], m[3])

def stop():
    drive_raw(0, 0, 0, 0)

# -----------------------------
# Tick-based motion primitives
# -----------------------------
DEFAULT_SPEED_CMD = 300
MOTION_TIMEOUT_S = 10.0

def forward_ticks(ticks: int, speed_cmd: int = DEFAULT_SPEED_CMD):
    ticks = abs(int(ticks))
    if ticks == 0:
        return

    start = _get_ticks()
    t0 = time.time()

    drive_raw(speed_cmd, speed_cmd, speed_cmd, speed_cmd)

    while True:
        now = _get_ticks()
        dL = 0.5 * ((now[0] - start[0]) + (now[3] - start[3]))
        dR = 0.5 * ((now[1] - start[1]) + (now[2] - start[2]))
        d = 0.5 * (dL + dR)

        if d >= ticks:
            break
        if (time.time() - t0) > MOTION_TIMEOUT_S:
            print("\n[WARN] forward_ticks timeout; stopping for safety.")
            break
        time.sleep(0.005)

    stop()

def rotate_ticks(ticks: int, speed_cmd: int = DEFAULT_SPEED_CMD):
    ticks = int(ticks)
    if ticks == 0:
        return

    target = abs(ticks)
    sign = 1 if ticks > 0 else -1

    start = _get_ticks()
    t0 = time.time()

    drive_raw(+sign * speed_cmd, -sign * speed_cmd, -sign * speed_cmd, +sign * speed_cmd)

    while True:
        now = _get_ticks()
        dL = 0.5 * ((now[0] - start[0]) + (now[3] - start[3]))
        dR = 0.5 * ((now[1] - start[1]) + (now[2] - start[2]))
        diff = abs(dR - dL)

        if diff >= target:
            break
        if (time.time() - t0) > MOTION_TIMEOUT_S:
            print("\n[WARN] rotate_ticks timeout; stopping for safety.")
            break
        time.sleep(0.005)

    stop()

# -----------------------------
# NEW — rotate by degrees
# -----------------------------
def rotate_degrees(deg: float, speed_cmd: int = DEFAULT_SPEED_CMD):
    """
    Convert degrees -> ticks using robot geometry + real tick distance.
    Positive deg = CCW, negative deg = CW.
    """
    theta = math.radians(deg)

    # distance one side travels for half the robot width
    ticks_per_side = (TRACK_WIDTH_M * 0.5 * abs(theta)) / METERS_PER_TICK

    # rotate_ticks() uses target = |dR - dL| = 2 * ticks_per_side
    diff_ticks_target = int(round(2.0 * ticks_per_side))

    rotate_ticks(diff_ticks_target if deg >= 0 else -diff_ticks_target, speed_cmd)

# Convenience forward functions
def forward_cm(cm: float, speed_cmd: int = DEFAULT_SPEED_CMD):
    ticks = int(round((cm / 100.0) * TICKS_PER_METER))
    forward_ticks(ticks, speed_cmd)

def forward_m(m: float, speed_cmd: int = DEFAULT_SPEED_CMD):
    ticks = int(round(m * TICKS_PER_METER))
    forward_ticks(ticks, speed_cmd)

# -----------------------------
# Main
# -----------------------------
def main():
    global running, x_m, y_m, theta_rad

    print("Initializing motor driver...")

    send_upload_command(1)  # total encoder ticks
    time.sleep(0.1)
    set_motor_parameter()
    time.sleep(0.1)

    running = True
    threading.Thread(target=rx_loop, daemon=True).start()
    threading.Thread(target=odom_loop, daemon=True).start()
    threading.Thread(target=print_loop, daemon=True).start()

    print("\nReady.")
    print("Commands:")
    print("  f <ticks> [speed]    -> forward by ticks")
    print("  fc <cm> [speed]      -> forward by centimeters")
    print("  fm <m> [speed]       -> forward by meters")
    print("  r <ticks> [speed]    -> rotate by ticks")
    print("  rd <deg> [speed]     -> rotate by degrees  <--- NEW")
    print("  s                    -> stop")
    print("  reset                -> reset pose")
    print("  q                    -> quit")

    try:
        while True:
            cmd = input("\n> ").strip().split()
            if not cmd:
                continue

            if cmd[0] == "f":
                ticks = int(cmd[1])
                speed = int(cmd[2]) if len(cmd) > 2 else DEFAULT_SPEED_CMD
                forward_ticks(ticks, speed)

            elif cmd[0] == "fc":
                cm = float(cmd[1])
                speed = int(cmd[2]) if len(cmd) > 2 else DEFAULT_SPEED_CMD
                forward_cm(cm, speed)

            elif cmd[0] == "fm":
                m = float(cmd[1])
                speed = int(cmd[2]) if len(cmd) > 2 else DEFAULT_SPEED_CMD
                forward_m(m, speed)

            elif cmd[0] == "r":
                ticks = int(cmd[1])
                speed = int(cmd[2]) if len(cmd) > 2 else DEFAULT_SPEED_CMD
                rotate_ticks(ticks, speed)

            elif cmd[0] == "rd":  # NEW
                deg = float(cmd[1])
                speed = int(cmd[2]) if len(cmd) > 2 else DEFAULT_SPEED_CMD
                rotate_degrees(deg, speed)

            elif cmd[0] == "s":
                stop()

            elif cmd[0] == "reset":
                x_m = y_m = theta_rad = 0.0
                print("Pose reset.")

            elif cmd[0] == "q":
                stop()
                break

            else:
                print("Unknown command.")

    finally:
        running = False
        stop()
        time.sleep(0.1)
        ser.close()
        print("\nStopped. Final:", pose_str(), "| ticks:", _get_ticks())

if __name__ == "__main__":
    main()
