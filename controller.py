import serial
import math
import time
import sys
import threading
from collections import deque

SERIAL_PORT = '/dev/serial0'
BAUD_RATE   = 115200

# YDLidar Configuration
LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUDRATE = 115200
LIDAR_PACKET_HEADER = 0xAA
LIDAR_DISTANCE_SCALE = 0.001  # mm to meters
LIDAR_POINTS_PER_PACKET = 12

WHEEL_BASE   = 31.5
MAX_LINEAR_VEL  = 50.0
MAX_ANGULAR_VEL =  2.0

Kp_linear  = 2.51
Kp_angular = 8.5
DISTANCE_THRESHOLD = 5.0
HEADING_THRESHOLD = 0.087

LOOP_HZ  = 10
LOOP_DT  = 1.0 / LOOP_HZ

# Shared LiDAR data storage (thread-safe)
lidar_data = deque(maxlen=1000)  # Store last 1000 points
lidar_lock = threading.Lock()

class LidarThread(threading.Thread):
    def __init__(self, port, baudrate):
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.running = False
        self.ser = None
        
    def parse_packet(self, packet_bytes):
        points = []
        for i in range(2, len(packet_bytes)-2, 3):
            dist_raw = packet_bytes[i] | (packet_bytes[i+1] << 8)
            quality = packet_bytes[i+2]
            distance = dist_raw * LIDAR_DISTANCE_SCALE
            if 0.02 < distance < 6.0:
                points.append((distance, quality))
            else:
                points.append((0, 0))
        return points
    
    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.running = True
            print(f"LiDAR connected on {self.port}")
            
            angle_step = 360 / LIDAR_POINTS_PER_PACKET
            angle_offset = 0
            
            while self.running:
                byte = self.ser.read(1)
                if not byte:
                    continue
                if byte[0] == LIDAR_PACKET_HEADER:
                    packet = self.ser.read(11)
                    if len(packet) != 11:
                        continue
                    full_packet = bytearray([LIDAR_PACKET_HEADER]) + packet
                    points = self.parse_packet(full_packet)
                    
                    with lidar_lock:
                        for i, (distance, quality) in enumerate(points):
                            angle = (angle_offset + i * angle_step) % 360
                            lidar_data.append((angle, distance, quality, time.time()))
                    
                    angle_offset = (angle_offset + len(points) * angle_step) % 360
                    
        except Exception as e:
            print(f"LiDAR error: {e}")
        finally:
            if self.ser:
                self.ser.close()
            print("LiDAR disconnected")
    
    def stop(self):
        self.running = False

def normalize_angle(angle):
    while angle >  math.pi: angle -= 2 * math.pi
    while angle < -math.pi: angle += 2 * math.pi
    return angle

def get_obstacles_in_cone(center_angle, cone_width=30, max_distance=1.0):
    """
    Get obstacles within a cone in front of the robot.
    center_angle: direction in degrees (0 = forward)
    cone_width: width of cone in degrees
    max_distance: maximum distance to consider (meters)
    Returns: list of (angle, distance) tuples
    """
    obstacles = []
    with lidar_lock:
        for angle, distance, quality, timestamp in lidar_data:
            if distance > 0 and distance < max_distance:
                angle_diff = abs((angle - center_angle + 180) % 360 - 180)
                if angle_diff < cone_width / 2:
                    obstacles.append((angle, distance))
    return obstacles

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
    
    # Start LiDAR thread
    lidar_thread = None
    try:
        lidar_thread = LidarThread(LIDAR_PORT, LIDAR_BAUDRATE)
        lidar_thread.start()
    except Exception as e:
        print(f"Warning: Could not start LiDAR: {e}")
    
    waypoint_index = 0
    target_x, target_y, desired_theta = waypoints[waypoint_index]
    position_reached = False
    orientation_reached = False
    all_waypoints_reached = False

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
            
            if not position_reached:
                desired_heading = math.atan2(dy, dx)
                heading_error = normalize_angle(desired_heading - fused_theta)
                v, omega, position_reached = go_to_goal(x, y, fused_theta, lv, rv)
                
                if position_reached and desired_theta is None:
                    orientation_reached = True
            
            elif not orientation_reached and desired_theta is not None:
                heading_error = normalize_angle(desired_theta - fused_theta)
                omega, orientation_reached = align_orientation(fused_theta, desired_theta)
                v = 0.0
            else:
                v, omega = 0.0, 0.0
                heading_error = 0.0
                
                if waypoint_index < len(waypoints) - 1:
                    waypoint_index += 1
                    wp = waypoints[waypoint_index]
                    target_x, target_y = wp[0], wp[1]
                    desired_theta = wp[2] if len(wp) == 3 else None
                    position_reached = False
                    orientation_reached = False
                else:
                    all_waypoints_reached = True

            status = "ALIGN" if position_reached and not orientation_reached else "MOVE"
            
            # Get LiDAR point count
            with lidar_lock:
                lidar_points = len(lidar_data)
            
            print(f"WP{waypoint_index+1}/{len(waypoints)} [{status}] Pos:({x:.1f},{y:.1f}) Goal:({target_x:.1f},{target_y:.1f}) "
                  f"Dist:{dist:.1f} Fused:{math.degrees(fused_theta):.1f}° "
                  f"Err:{math.degrees(heading_error):.1f}° v:{v:.1f} ω:{omega:.2f} LiDAR:{lidar_points}")

            cmd = f"v:{v:.3f},w:{omega:.4f}\n"
            ser.write(cmd.encode())

            elapsed = time.time() - loop_start
            time.sleep(max(0.0, LOOP_DT - elapsed))
    except KeyboardInterrupt:
        ser.write(b'v:0.0,w:0.0\n')
        time.sleep(0.2)
    finally:
        ser.close()
        if lidar_thread:
            lidar_thread.stop()
            lidar_thread.join(timeout=2)

if __name__=="__main__":
    if len(sys.argv) < 3:
        print("Usage: python controller.py <x1> <y1> [theta1] [<x2> <y2> [theta2] ...]")
        print("Examples:")
        print("  python controller.py 100 0")
        print("  python controller.py 100 0 45")
        print("  python controller.py 50 50 100 0 45 50 -50 0")
        sys.exit(1)
    
    waypoints = []
    args = [float(arg) for arg in sys.argv[1:]]
    
    i = 0
    while i < len(args):
        if i + 1 >= len(args):
            print("Error: Each waypoint needs at least x and y coordinates")
            sys.exit(1)
        
        x = args[i]
        y = args[i + 1]
        
        has_theta = False
        if i + 2 < len(args):
            remaining_after_theta = len(args) - (i + 3)
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