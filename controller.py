import serial
import math
import time
import sys

# ── Serial port to ESP32 ────────────────────────────────────────────────────
SERIAL_PORT = '/dev/serial0'   # Pi Zero UART (GPIO 14 TX, GPIO 15 RX)
BAUD_RATE   = 115200

# ── Robot parameters (must match firmware) ──────────────────────────────────
WHEEL_BASE   = 31.5   # cm
MAX_LINEAR_VEL  = 50.0   # cm/s
MAX_ANGULAR_VEL =  2.0   # rad/s

# ── Go-to-goal gains ────────────────────────────────────────────────────────
Kp_linear  = 0.5
Kp_angular = 1.5
DISTANCE_THRESHOLD = 10.0  # cm

# ── Loop rate ───────────────────────────────────────────────────────────────
LOOP_HZ  = 10
LOOP_DT  = 1.0 / LOOP_HZ


def normalize_angle(angle):
    while angle >  math.pi: angle -= 2 * math.pi
    while angle < -math.pi: angle += 2 * math.pi
    return angle


def go_to_goal(x, y, mpu_theta):
    """Returns (v cm/s, omega rad/s, goal_reached bool)."""
    dx = target_x - x
    dy = target_y - y
    distance = math.sqrt(dx*dx + dy*dy)

    if distance < DISTANCE_THRESHOLD:
        return 0.0, 0.0, True

    v = Kp_linear * distance
    v = max(0.0, min(v, MAX_LINEAR_VEL))

    desired_heading = math.atan2(dy, dx)
    heading_error   = normalize_angle(desired_heading - mpu_theta)

    omega = Kp_angular * heading_error
    omega = max(-MAX_ANGULAR_VEL, min(omega, MAX_ANGULAR_VEL))

    return v, omega, False


def parse_state(line):
    """
    Parse ESP32 state line: x,y,theta,mpu_angle,left_vel,right_vel
    Returns (x, y, theta, mpu_angle, lv, rv) or None on error.
    """
    try:
        parts = line.strip().split(',')
        if len(parts) != 6:
            return None
        return tuple(float(p) for p in parts)
    except ValueError:
        return None


def main(target_x, target_y):
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)
    print(f"Connected to ESP32 on {SERIAL_PORT}")
    print(f"Target: ({target_x:.1f}, {target_y:.1f}) cm")
    time.sleep(1)

    goal_reached = False

    while True:
        loop_start = time.time()

        # ── Read state from ESP32 ───────────────────────────────────────────
        state = None
        raw = ser.readline().decode('utf-8', errors='ignore')
        if raw:
            state = parse_state(raw)

        if state is None:
            # No valid data yet — send zero command to keep watchdog happy
            ser.write(b'v:0.0,w:0.0\n')
            time.sleep(max(0.0, LOOP_DT - (time.time() - loop_start)))
            continue

        x, y, theta, mpu_angle, lv, rv = state

        print(f"x={x:.1f} y={y:.1f} θ={math.degrees(mpu_angle):.1f}° "
              f"lv={lv:.1f} rv={rv:.1f}")

        # ── Go-to-goal ──────────────────────────────────────────────────────
        if not goal_reached:
            v, omega, goal_reached = go_to_goal(x, y, mpu_angle)
        else:
            v, omega = 0.0, 0.0
            print("Goal reached!")

        # ── Send (v, ω) to ESP32 ────────────────────────────────────────────
        cmd = f"v:{v:.3f},w:{omega:.4f}\n"
        ser.write(cmd.encode())

        # ── Hold loop rate ───────────────────────────────────────────────────
        elapsed = time.time() - loop_start
        time.sleep(max(0.0, LOOP_DT - elapsed))


if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Usage: python controller.py <target_x> <target_y>")
        print("Example: python controller.py 200 200")
        sys.exit(1)
    
    try:
        target_x = float(sys.argv[1])
        target_y = float(sys.argv[2])
    except ValueError:
        print("Error: target_x and target_y must be numbers")
        sys.exit(1)
    
    try:
        main(target_x, target_y)
    except KeyboardInterrupt:
        print("\nStopped.")
