import serial
import math
import time
import sys

SERIAL_PORT = '/dev/serial0'
BAUD_RATE   = 115200

WHEEL_BASE   = 31.5
MAX_LINEAR_VEL  = 50.0
MAX_ANGULAR_VEL =  2.0

Kp_linear  = 2.51
Kp_angular = 8.5
DISTANCE_THRESHOLD = 5.0

LOOP_HZ  = 10
LOOP_DT  = 1.0 / LOOP_HZ

def normalize_angle(angle):
    while angle >  math.pi: angle -= 2 * math.pi
    while angle < -math.pi: angle += 2 * math.pi
    return angle

def go_to_goal(x, y, fused_theta, lv, rv):
    dx = target_x - x
    dy = target_y - y
    distance = math.sqrt(dx*dx + dy*dy)

    if distance < DISTANCE_THRESHOLD:
        return 0.0, 0.0, True

    v = Kp_linear*distance
    v = max(0.0, min(v, MAX_LINEAR_VEL))

    desired_heading = math.atan2(dy, dx)
    heading_error = normalize_angle(desired_heading - fused_theta)
    omega = Kp_angular*heading_error
    omega = max(-MAX_ANGULAR_VEL, min(omega, MAX_ANGULAR_VEL))
    return v, omega, False

def parse_state(line):
    try:
        parts = line.strip().split(',')
        if len(parts)!=6: return None
        return tuple(float(p) for p in parts)
    except: return None

def main(waypoints):
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)
    global target_x, target_y
    
    waypoint_index = 0
    target_x, target_y = waypoints[waypoint_index]
    goal_reached = False
    all_waypoints_reached = False
    
    print(f"Navigating through {len(waypoints)} waypoint(s)")
    for i, (wx, wy) in enumerate(waypoints):
        print(f"  Waypoint {i+1}: ({wx:.1f}, {wy:.1f})")
    print()

    try:
        while not all_waypoints_reached:
            loop_start = time.time()
            raw = ser.readline().decode('utf-8', errors='ignore')
            state = parse_state(raw) if raw else None

            if state is None:
                cmd = b'v:0.0,w:0.0\n'
                ser.write(cmd)
                time.sleep(max(0.0, LOOP_DT - (time.time()-loop_start)))
                continue

            x, y, fused_theta, mpu_angle, lv, rv = state
            
            dx = target_x - x
            dy = target_y - y
            dist = math.sqrt(dx*dx + dy*dy)
            desired_heading = math.atan2(dy, dx)
            heading_error = normalize_angle(desired_heading - fused_theta)
            
            if not goal_reached:
                v, omega, goal_reached = go_to_goal(x, y, fused_theta, lv, rv)
            else:
                v, omega = 0.0, 0.0
                
                # Check if there are more waypoints
                if waypoint_index < len(waypoints) - 1:
                    waypoint_index += 1
                    target_x, target_y = waypoints[waypoint_index]
                    goal_reached = False
                    print(f"\n→ Moving to waypoint {waypoint_index+1}: ({target_x:.1f}, {target_y:.1f})\n")
                else:
                    print(f"\n✓ All waypoints reached!\n")
                    all_waypoints_reached = True

            print(f"WP{waypoint_index+1}/{len(waypoints)} Pos:({x:.1f},{y:.1f}) Goal:({target_x:.1f},{target_y:.1f}) "
                  f"Dist:{dist:.1f} Fused:{math.degrees(fused_theta):.1f}° "
                  f"Err:{math.degrees(heading_error):.1f}° v:{v:.1f} ω:{omega:.2f}")

            cmd = f"v:{v:.3f},w:{omega:.4f}\n"
            ser.write(cmd.encode())

            elapsed = time.time() - loop_start
            time.sleep(max(0.0, LOOP_DT - elapsed))
    except KeyboardInterrupt:
        print("\nStopping robot...")
        ser.write(b'v:0.0,w:0.0\n')
        time.sleep(0.2)
        print("Robot stopped.")
    finally:
        ser.close()

if __name__=="__main__":
    if len(sys.argv) < 3 or len(sys.argv) % 2 == 0:
        print("Usage: python controller.py <x1> <y1> [<x2> <y2> ...]")
        print("Examples:")
        print("  python controller.py 100 0              # Single waypoint")
        print("  python controller.py 50 50 100 0 50 -50 # Three waypoints")
        sys.exit(1)
    
    # Parse waypoints from command line arguments
    waypoints = []
    for i in range(1, len(sys.argv), 2):
        x = float(sys.argv[i])
        y = float(sys.argv[i+1])
        waypoints.append((x, y))
    
    main(waypoints)