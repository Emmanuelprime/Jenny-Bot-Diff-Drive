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
HEADING_THRESHOLD = 0.087  # ~5 degrees in radians

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

def align_orientation(fused_theta, desired_theta):
    """Pure rotation to align with desired orientation"""
    heading_error = normalize_angle(desired_theta - fused_theta)
    
    if abs(heading_error) < HEADING_THRESHOLD:
        return 0.0, True
    
    omega = Kp_angular * heading_error
    omega = max(-MAX_ANGULAR_VEL, min(omega, MAX_ANGULAR_VEL))
    return omega, False

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
    target_x, target_y, desired_theta = waypoints[waypoint_index]
    position_reached = False
    orientation_reached = False
    all_waypoints_reached = False
    
    print(f"Navigating through {len(waypoints)} waypoint(s)")
    for i, wp in enumerate(waypoints):
        wx, wy, wtheta = wp
        if wtheta is not None:
            print(f"  Waypoint {i+1}: ({wx:.1f}, {wy:.1f}, {math.degrees(wtheta):.1f}°)")
        else:
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
            
            # Two-phase control: position first, then orientation
            if not position_reached:
                desired_heading = math.atan2(dy, dx)
                heading_error = normalize_angle(desired_heading - fused_theta)
                v, omega, position_reached = go_to_goal(x, y, fused_theta, lv, rv)
                
                if position_reached:
                    print(f"  → Position reached!")
                    # Check if waypoint has desired orientation
                    if desired_theta is not None:
                        print(f"  → Aligning to {math.degrees(desired_theta):.1f}°...")
                    else:
                        orientation_reached = True
            
            elif not orientation_reached and desired_theta is not None:
                # Pure rotation to desired orientation
                heading_error = normalize_angle(desired_theta - fused_theta)
                omega, orientation_reached = align_orientation(fused_theta, desired_theta)
                v = 0.0
                
                if orientation_reached:
                    print(f"  → Orientation aligned!")
            else:
                v, omega = 0.0, 0.0
                heading_error = 0.0
                
                # Check if there are more waypoints
                if waypoint_index < len(waypoints) - 1:
                    waypoint_index += 1
                    wp = waypoints[waypoint_index]
                    target_x, target_y = wp[0], wp[1]
                    desired_theta = wp[2] if len(wp) == 3 else None
                    position_reached = False
                    orientation_reached = False
                    print(f"\n→ Moving to waypoint {waypoint_index+1}\n")
                else:
                    print(f"\n✓ All waypoints reached!\n")
                    all_waypoints_reached = True

            # Status display
            status = "ALIGN" if position_reached and not orientation_reached else "MOVE"
            print(f"WP{waypoint_index+1}/{len(waypoints)} [{status}] Pos:({x:.1f},{y:.1f}) Goal:({target_x:.1f},{target_y:.1f}) "
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
    if len(sys.argv) < 3:
        print("Usage: python controller.py <x1> <y1> [theta1] [<x2> <y2> [theta2] ...]")
        print("Examples:")
        print("  python controller.py 100 0                    # Single waypoint (x, y)")
        print("  python controller.py 100 0 45                 # With orientation (45°)")
        print("  python controller.py 50 50 100 0 45 50 -50 0  # Three waypoints with final orientations")
        sys.exit(1)
    
    # Parse waypoints from command line arguments
    waypoints = []
    args = [float(arg) for arg in sys.argv[1:]]
    
    i = 0
    while i < len(args):
        if i + 1 >= len(args):
            print("Error: Each waypoint needs at least x and y coordinates")
            sys.exit(1)
        
        x = args[i]
        y = args[i + 1]
        
        # Check if there's a third value that could be theta
        # It's theta if: we have a 3rd value AND (we're at the end OR there are 2+ more values after)
        has_theta = False
        if i + 2 < len(args):
            remaining_after_theta = len(args) - (i + 3)
            # If remaining values is 0 or even (divisible by 2), this is a theta
            if remaining_after_theta == 0 or remaining_after_theta >= 2:
                has_theta = True
        
        if has_theta:
            theta_deg = args[i + 2]
            theta_rad = math.radians(theta_deg)
            waypoints.append((x, y, theta_rad))
            i += 3
        else:
            waypoints.append((x, y, None))
            i += 2
    
    main(waypoints)