import serial
import math
import time
import sys
import threading
from collections import deque
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation

SERIAL_PORT = '/dev/serial0'
BAUD_RATE   = 115200

# YDLidar Configuration
LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUDRATE = 115200
LIDAR_DISTANCE_SCALE = 0.001  # mm to meters (internal conversion, output is in cm)
LIDAR_ANGLE_OFFSET = 60  # degrees - offset to align LiDAR front with robot front

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
LIDAR_DATA_TIMEOUT = 2.0  # seconds

# Plotting data
path_history = deque(maxlen=1000)  # Store robot path (x, y, theta)
plot_lock = threading.Lock()

class LidarThread(threading.Thread):
    def __init__(self, port, baudrate):
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.running = False
        self.ser = None
        
    def parse_packet(self, packet_bytes):
        if len(packet_bytes) < 10:
            return None, []
        
        lsn = packet_bytes[3]
        
        fsa_raw = packet_bytes[4] | (packet_bytes[5] << 8)
        lsa_raw = packet_bytes[6] | (packet_bytes[7] << 8)
        
        start_angle = (fsa_raw >> 1) / 64.0
        end_angle = (lsa_raw >> 1) / 64.0
        
        angle_diff = end_angle - start_angle
        if angle_diff < 0:
            angle_diff += 360
        
        angle_step = angle_diff / (lsn - 1) if lsn > 1 else 0
        
        points = []
        data_start = 10
        
        for i in range(lsn):
            idx = data_start + i * 3
            if idx + 2 < len(packet_bytes):
                dist_raw = packet_bytes[idx] | (packet_bytes[idx+1] << 8)
                quality = packet_bytes[idx+2]
                distance_m = dist_raw * 0.25 * LIDAR_DISTANCE_SCALE
                distance_cm = distance_m * 100
                angle = (start_angle + i * angle_step + LIDAR_ANGLE_OFFSET) % 360
                
                if 2 < distance_cm < 600:
                    points.append((angle, distance_cm, quality))
        
        return start_angle, points
    
    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.running = True
            print(f"LiDAR connected on {self.port}")
            
            while self.running:
                byte = self.ser.read(1)
                if not byte:
                    continue
                
                if byte[0] == 0xAA:
                    second_byte = self.ser.read(1)
                    if not second_byte or second_byte[0] != 0x55:
                        continue
                    
                    header = self.ser.read(8)
                    if len(header) != 8:
                        continue
                    
                    lsn = header[1]
                    data_bytes = lsn * 3
                    data = self.ser.read(data_bytes)
                    if len(data) != data_bytes:
                        continue
                    
                    full_packet = bytearray([0xAA, 0x55]) + header + data
                    start_angle, points = self.parse_packet(full_packet)
                    
                    if start_angle is not None:
                        with lidar_lock:
                            for angle, distance, quality in points:
                                lidar_data.append((angle, distance, quality, time.time()))
                    
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

def get_obstacles_in_cone(center_angle, cone_width=30, max_distance=100):
    """Get obstacles within a cone in front of the robot."""
    obstacles = []
    current_time = time.time()
    
    with lidar_lock:
        for angle, distance, quality, timestamp in lidar_data:
            if current_time - timestamp > LIDAR_DATA_TIMEOUT:
                continue
            
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

# Global variables for navigation state (accessed by both threads)
current_state = {'x': 0, 'y': 0, 'theta': 0, 'status': 'INIT', 'waypoint_idx': 0}
state_lock = threading.Lock()
controller_running = False

def controller_thread(waypoints, use_lidar=False):
    """Background thread that handles robot navigation"""
    global controller_running, target_x, target_y, current_state
    
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)
    
    # Start LiDAR thread only if requested
    lidar_thread = None
    if use_lidar:
        try:
            lidar_thread = LidarThread(LIDAR_PORT, LIDAR_BAUDRATE)
            lidar_thread.start()
        except Exception as e:
            print(f"Warning: Could not start LiDAR: {e}")
    else:
        print("LiDAR disabled (use --lidar to enable)")
    
    waypoint_index = 0
    target_x, target_y, desired_theta = waypoints[waypoint_index]
    position_reached = False
    orientation_reached = False
    all_waypoints_reached = False

    try:
        while controller_running and not all_waypoints_reached:
            loop_start = time.time()
            raw = ser.readline().decode('utf-8', errors='ignore')
            state = parse_state(raw) if raw else None

            if state is None:
                cmd = b'v:0.0,w:0.0\n'
                ser.write(cmd)
                time.sleep(max(0.0, LOOP_DT - (time.time()-loop_start)))
                continue

            x, y, fused_theta, mpu_angle, lv, rv = state
            
            # Update plot data
            with plot_lock:
                path_history.append((x, y, fused_theta))
            
            with state_lock:
                current_state['x'] = x
                current_state['y'] = y
                current_state['theta'] = fused_theta
                current_state['waypoint_idx'] = waypoint_index
            
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
            
            with state_lock:
                current_state['status'] = status
            
            # Get LiDAR point count if enabled
            lidar_status = ""
            if use_lidar:
                current_time = time.time()
                with lidar_lock:
                    fresh_points = sum(1 for _, _, _, ts in lidar_data 
                                      if current_time - ts <= LIDAR_DATA_TIMEOUT)
                lidar_status = f" LiDAR:{fresh_points}"
            
            print(f"WP{waypoint_index+1}/{len(waypoints)} [{status}] Pos:({x:.1f},{y:.1f}) Goal:({target_x:.1f},{target_y:.1f}) "
                  f"Dist:{dist:.1f} Fused:{math.degrees(fused_theta):.1f}° "
                  f"Err:{math.degrees(heading_error):.1f}° v:{v:.1f} ω:{omega:.2f}{lidar_status}")

            cmd = f"v:{v:.3f},w:{omega:.4f}\n"
            ser.write(cmd.encode())

            elapsed = time.time() - loop_start
            time.sleep(max(0.0, LOOP_DT - elapsed))
            
    except Exception as e:
        print(f"Controller error: {e}")
    finally:
        ser.write(b'v:0.0,w:0.0\n')
        time.sleep(0.2)
        ser.close()
        if lidar_thread:
            lidar_thread.stop()
            lidar_thread.join(timeout=2)
        controller_running = False

def update_plot(frame, ax, waypoints, robot_marker, path_line, waypoint_scatter, status_text):
    """Update function for matplotlib animation"""
    
    with plot_lock:
        path = list(path_history)
    
    with state_lock:
        state = current_state.copy()
    
    if not path:
        return robot_marker, path_line, status_text
    
    # Update path trail
    if len(path) > 0:
        x_coords = [p[0] for p in path]
        y_coords = [p[1] for p in path]
        path_line.set_data(x_coords, y_coords)
    
    # Update robot position and orientation
    if len(path) > 0:
        x, y, theta = path[-1]
        
        # Robot as a triangle pointing in direction of theta
        robot_size = 8  # cm
        # Triangle vertices in robot frame
        triangle = [
            (robot_size, 0),      # Front point
            (-robot_size/2, robot_size/2),   # Back left
            (-robot_size/2, -robot_size/2)   # Back right
        ]
        
        # Rotate and translate
        rotated = []
        for px, py in triangle:
            rx = px * math.cos(theta) - py * math.sin(theta) + x
            ry = px * math.sin(theta) + py * math.cos(theta) + y
            rotated.append((rx, ry))
        
        robot_marker.set_xy(rotated)
    
    # Update status text
    wp_idx = state['waypoint_idx']
    status_str = state['status']
    status_text.set_text(f"Waypoint: {wp_idx+1}/{len(waypoints)} | Status: {status_str}\n"
                        f"Position: ({state['x']:.1f}, {state['y']:.1f}) cm\n"
                        f"Heading: {math.degrees(state['theta']):.1f}°")
    
    return robot_marker, path_line, waypoint_scatter, status_text

def main_with_plot(waypoints, use_lidar=False):
    """Main function with live plotting"""
    global controller_running
    
    # Set up the plot
    fig, ax = plt.subplots(figsize=(10, 10))
    
    # Calculate plot bounds from waypoints
    wp_x = [wp[0] for wp in waypoints]
    wp_y = [wp[1] for wp in waypoints]
    margin = 50  # cm
    x_min = min(0, min(wp_x)) - margin
    x_max = max(0, max(wp_x)) + margin
    y_min = min(0, min(wp_y)) - margin
    y_max = max(0, max(wp_y)) + margin
    
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('X (cm) - Forward')
    ax.set_ylabel('Y (cm) - Left')
    ax.set_title('Robot Path Visualization')
    
    # Plot waypoints
    wp_x = [wp[0] for wp in waypoints]
    wp_y = [wp[1] for wp in waypoints]
    waypoint_scatter = ax.scatter(wp_x, wp_y, c='red', s=100, marker='x', 
                                  linewidths=2, label='Waypoints', zorder=5)
    
    # Label waypoints
    for i, (x, y) in enumerate(zip(wp_x, wp_y), 1):
        ax.annotate(f'WP{i}', (x, y), xytext=(5, 5), textcoords='offset points',
                   fontsize=10, color='red', fontweight='bold')
    
    # Plot starting position
    ax.plot(0, 0, 'go', markersize=12, label='Start', zorder=5)
    
    # Initialize robot marker (triangle)
    robot_marker = patches.Polygon([(0, 0), (0, 0), (0, 0)], 
                                  closed=True, color='blue', alpha=0.7, zorder=10)
    ax.add_patch(robot_marker)
    
    # Initialize path line
    path_line, = ax.plot([], [], 'b-', linewidth=2, alpha=0.6, label='Path')
    
    # Status text
    status_text = ax.text(0.02, 0.98, '', transform=ax.transAxes, 
                         verticalalignment='top', fontfamily='monospace',
                         bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
                         fontsize=9)
    
    ax.legend(loc='upper right')
    
    # Start controller in background thread
    controller_running = True
    ctrl_thread = threading.Thread(target=controller_thread, 
                                   args=(waypoints, use_lidar), 
                                   daemon=True)
    ctrl_thread.start()
    
    # Set up animation
    ani = FuncAnimation(fig, update_plot, 
                       fargs=(ax, waypoints, robot_marker, path_line, 
                             waypoint_scatter, status_text),
                       interval=100, blit=False, cache_frame_data=False)
    
    print("\nLive plot window opened. Close window to stop navigation.")
    
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        controller_running = False
        ctrl_thread.join(timeout=2)
        print("\nNavigation stopped.")

if __name__=="__main__":
    if len(sys.argv) < 3:
        print("Usage: python controller_plotted.py [--lidar] <x1> <y1> [theta1] [<x2> <y2> [theta2] ...]")
        print("Options:")
        print("  --lidar    Enable LiDAR scanning (optional)")
        print()
        print("Examples:")
        print("  python controller_plotted.py 100 0")
        print("  python controller_plotted.py 100 0 45")
        print("  python controller_plotted.py --lidar 100 0")
        print("  python controller_plotted.py 50 50 100 0 45 50 -50 0")
        sys.exit(1)
    
    # Check for --lidar flag
    use_lidar = False
    args_list = sys.argv[1:]
    if '--lidar' in args_list:
        use_lidar = True
        args_list.remove('--lidar')
    
    waypoints = []
    args = [float(arg) for arg in args_list]
    
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
    
    # Run with live plotting
    main_with_plot(waypoints, use_lidar=use_lidar)
