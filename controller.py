import serial
import math
import time
import sys

SERIAL_PORT = '/dev/serial0'
BAUD_RATE   = 115200

WHEEL_BASE   = 31.5
MAX_LINEAR_VEL  = 50.0
MAX_ANGULAR_VEL =  2.0

Kp_linear  = 1.0  # Increased for faster movement
Kp_angular = 1.5
DISTANCE_THRESHOLD = 5.0

LOOP_HZ  = 10
LOOP_DT  = 1.0 / LOOP_HZ

def normalize_angle(angle):
    while angle >  math.pi: angle -= 2 * math.pi
    while angle < -math.pi: angle += 2 * math.pi
    return angle

def go_to_goal(x, y, mpu_theta, lv, rv):
    dx = target_x - x
    dy = target_y - y
    distance = math.sqrt(dx*dx + dy*dy)

    if distance < DISTANCE_THRESHOLD and abs(lv)<2.0 and abs(rv)<2.0:
        return 0.0, 0.0, True

    # smooth slowdown
    v = Kp_linear*distance
    # if distance < 50.0: v *= distance/50.0
    v = max(0.0, min(v, MAX_LINEAR_VEL))

    desired_heading = math.atan2(dy, dx)
    heading_error = normalize_angle(desired_heading - mpu_theta)
    omega = Kp_angular*heading_error
    omega = max(-MAX_ANGULAR_VEL, min(omega, MAX_ANGULAR_VEL))
    return v, omega, False

def parse_state(line):
    try:
        parts = line.strip().split(',')
        if len(parts)!=6: return None
        return tuple(float(p) for p in parts)
    except: return None

def main(target_x_arg, target_y_arg):
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)
    global target_x, target_y
    target_x = target_x_arg
    target_y = target_y_arg
    goal_reached = False

    while True:
        loop_start = time.time()
        raw = ser.readline().decode('utf-8', errors='ignore')
        state = parse_state(raw) if raw else None

        if state is None:
            cmd = b'v:0.0,w:0.0\n'
            ser.write(cmd)
            time.sleep(max(0.0, LOOP_DT - (time.time()-loop_start)))
            continue

        x, y, theta, mpu_angle, lv, rv = state
        
        # Calculate distance and heading for debugging
        dx = target_x - x
        dy = target_y - y
        dist = math.sqrt(dx*dx + dy*dy)
        desired_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(desired_heading - mpu_angle)
        
        if not goal_reached:
            v, omega, goal_reached = go_to_goal(x, y, mpu_angle, lv, rv)
        else:
            v, omega = 0.0, 0.0

        # Print debug info
        print(f"Pos:({x:.1f},{y:.1f}) Goal:({target_x:.1f},{target_y:.1f}) "
              f"Dist:{dist:.1f} Head:{math.degrees(mpu_angle):.1f}° "
              f"Err:{math.degrees(heading_error):.1f}° v:{v:.1f} ω:{omega:.2f}")

        cmd = f"v:{v:.3f},w:{omega:.4f}\n"
        ser.write(cmd.encode())

        elapsed = time.time() - loop_start
        time.sleep(max(0.0, LOOP_DT - elapsed))

if __name__=="__main__":
    if len(sys.argv)!=3:
        print("Usage: python controller.py <target_x> <target_y>")
        sys.exit(1)
    target_x = float(sys.argv[1])
    target_y = float(sys.argv[2])
    try: main(target_x, target_y)
    except KeyboardInterrupt: print("\nStopped.")