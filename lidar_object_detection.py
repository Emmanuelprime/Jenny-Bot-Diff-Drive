import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle
from collections import deque
import math
import time
# ----------------------------
# Configuration
# ----------------------------
PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
DISTANCE_SCALE = 0.001  # mm to meters (internal)
MAX_POINTS = 1500
MAX_DISTANCE = 300.0  # cm (3 meters for display)
UPDATE_INTERVAL = 30  # milliseconds - faster refresh
PACKETS_PER_UPDATE = 5  # Read more packets per frame

# Object detection parameters
CLUSTER_DISTANCE_THRESHOLD = 15  # cm - points within this distance are same object
MIN_CLUSTER_SIZE = 3  # minimum points to be considered an object
MAX_CLUSTER_SIZE = 100  # maximum points in a cluster

# Field of view (front cone)
FRONT_CONE_ANGLE = 60  # degrees - only process points in this cone (0° = forward)
FRONT_CONE_CENTER = 0  # degrees - center of the cone

# Robot coordinate system:
#     +Y (90°, Left)
#      |
#      |
#      +---- +X (0°, Front)
#     /
#    / Robot

# Store recent points
point_buffer = deque(maxlen=MAX_POINTS)
LIDAR_DATA_TIMEOUT = 0.5  # seconds - data older than this is stale

# ----------------------------
# Parse LiDAR packet (X3 protocol)
# ----------------------------
def parse_packet(packet_bytes):
    if len(packet_bytes) < 10:
        return None, []
    
    lsn = packet_bytes[3]  # Number of samples
    
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
            distance_m = dist_raw * 0.25 * DISTANCE_SCALE  # meters
            distance_cm = distance_m * 100  # convert to cm
            angle = (start_angle + i * angle_step) % 360
            
            if 2 < distance_cm < MAX_DISTANCE:  # 2cm to MAX_DISTANCE
                points.append((angle, distance_cm, quality))
    
    return start_angle, points

# ----------------------------
# Filter points to front cone
# ----------------------------
def filter_front_cone(points, center_angle=FRONT_CONE_CENTER, cone_width=FRONT_CONE_ANGLE):
    """
    Filter points to only include those in the front cone.
    center_angle: direction in degrees (0 = forward)
    cone_width: total width of cone in degrees
    """
    filtered = []
    half_cone = cone_width / 2.0
    current_time = time.time()
    
    for angle, distance, timestamp in points:
        # Skip stale data
        if current_time - timestamp > LIDAR_DATA_TIMEOUT:
            continue
        
        # Calculate angle difference (handle wraparound)
        angle_diff = abs((angle - center_angle + 180) % 360 - 180)
        if angle_diff <= half_cone:
            filtered.append((angle, distance))
    
    return filtered

# ----------------------------
# Read LiDAR data
# ----------------------------
def read_lidar_data(ser):
    packets_read = 0
    current_time = time.time()
    
    while packets_read < PACKETS_PER_UPDATE:  # Read multiple packets for faster updates
        byte = ser.read(1)
        if not byte:
            continue
        
        if byte[0] == 0xAA:
            second_byte = ser.read(1)
            if not second_byte or second_byte[0] != 0x55:
                continue
            
            header = ser.read(8)
            if len(header) != 8:
                continue
            
            lsn = header[1]
            data_bytes = lsn * 3
            data = ser.read(data_bytes)
            if len(data) != data_bytes:
                continue
            
            full_packet = bytearray([0xAA, 0x55]) + header + data
            start_angle, points = parse_packet(full_packet)
            
            if start_angle is not None:
                for angle, distance, quality in points:
                    point_buffer.append((angle, distance, current_time))
            
            packets_read += 1

# ----------------------------
# Convert polar to Cartesian (in cm) - Robot frame
# ----------------------------
def polar_to_cartesian(angle_deg, distance_cm):
    """
    Convert LiDAR polar coordinates to robot frame:
    - 0° (forward) = +X axis
    - 90° (left) = +Y axis
    - Turning left is positive rotation
    """
    angle_rad = math.radians(angle_deg)
    x = distance_cm * math.cos(angle_rad)  # Front/back
    y = distance_cm * math.sin(angle_rad)  # Left/right
    return x, y

# ----------------------------
# Cluster points into objects
# ----------------------------
def detect_objects():
    if len(point_buffer) == 0:
        return []
    
    # Filter to front cone only
    front_points = filter_front_cone(list(point_buffer))
    
    if len(front_points) == 0:
        return []
    
    # Convert polar to Cartesian
    points_xy = []
    for angle, distance in front_points:
        x, y = polar_to_cartesian(angle, distance)
        points_xy.append((x, y))
    
    # Simple clustering by distance
    clusters = []
    visited = [False] * len(points_xy)
    
    for i, (x1, y1) in enumerate(points_xy):
        if visited[i]:
            continue
        
        # Start new cluster
        cluster = [(x1, y1)]
        visited[i] = True
        
        # Find nearby points
        for j, (x2, y2) in enumerate(points_xy):
            if visited[j]:
                continue
            
            # Check distance to any point in current cluster
            for cx, cy in cluster:
                dist = math.sqrt((x2 - cx)**2 + (y2 - cy)**2)
                if dist < CLUSTER_DISTANCE_THRESHOLD:
                    cluster.append((x2, y2))
                    visited[j] = True
                    break
        
        # Only keep clusters of reasonable size
        if MIN_CLUSTER_SIZE <= len(cluster) <= MAX_CLUSTER_SIZE:
            clusters.append(cluster)
    
    # Calculate object properties
    objects = []
    for cluster in clusters:
        xs = [x for x, y in cluster]
        ys = [y for x, y in cluster]
        
        center_x = sum(xs) / len(xs)
        center_y = sum(ys) / len(ys)
        
        # Calculate bounding circle
        max_radius = 0
        for x, y in cluster:
            radius = math.sqrt((x - center_x)**2 + (y - center_y)**2)
            max_radius = max(max_radius, radius)
        
        distance_from_robot = math.sqrt(center_x**2 + center_y**2)
        angle_from_robot = math.degrees(math.atan2(center_y, center_x))  # atan2(y, x) for robot frame
        
        objects.append({
            'center': (center_x, center_y),
            'radius': max_radius,
            'points': len(cluster),
            'distance': distance_from_robot,
            'angle': angle_from_robot
        })
    
    return objects

# ----------------------------
# Matplotlib setup
# ----------------------------
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))

# Left: XY view with objects
ax1.set_xlim(-MAX_DISTANCE, MAX_DISTANCE)
ax1.set_ylim(-MAX_DISTANCE, MAX_DISTANCE)
ax1.set_aspect('equal')
ax1.grid(True, alpha=0.3)
ax1.set_xlabel('X (cm) - Front/Back')
ax1.set_ylabel('Y (cm) - Left/Right')
ax1.set_title('Object Detection - Robot Frame (Front=+X, Left=+Y)')

# Draw robot at center (15cm radius)
robot_circle = Circle((0, 0), 15, color='red', fill=True, alpha=0.5)
ax1.add_patch(robot_circle)
# Arrow pointing in +X direction (forward)
ax1.arrow(0, 0, 30, 0, head_width=10, head_length=10, fc='red', ec='red')
# Label for orientation
ax1.text(35, 0, 'FRONT', fontsize=8, color='red', fontweight='bold')

# Draw field of view cone
import matplotlib.patches as mpatches
# Cone centered at 0° (+X axis), spanning ±30°
theta1 = -FRONT_CONE_ANGLE/2  # -30°
theta2 = FRONT_CONE_ANGLE/2   # +30°
wedge = mpatches.Wedge((0, 0), MAX_DISTANCE, theta1, theta2, 
                       alpha=0.1, color='yellow', label='Detection Zone')
ax1.add_patch(wedge)

scatter = ax1.scatter([], [], c='blue', s=5, alpha=0.4, label='LiDAR points')
object_circles = []

# Right: Object list
ax2.axis('off')
ax2.set_xlim(0, 1)
ax2.set_ylim(0, 1)
object_text = ax2.text(0.05, 0.95, '', verticalalignment='top', fontfamily='monospace', fontsize=10)

# ----------------------------
# Animation update function
# ----------------------------
def update(frame):
    read_lidar_data(ser)
    
    # Clear previous object circles
    for circle in object_circles:
        circle.remove()
    object_circles.clear()
    
    if len(point_buffer) > 0:
        # Filter to front cone for both display and detection
        front_points = filter_front_cone(list(point_buffer))
        
        # Plot only front cone points
        x_coords = []
        y_coords = []
        
        for angle_deg, distance in front_points:
            x, y = polar_to_cartesian(angle_deg, distance)
            x_coords.append(x)
            y_coords.append(y)
        
        scatter.set_offsets(np.c_[x_coords, y_coords])
        
        # Detect objects (uses same filtered points internally)
        objects = detect_objects()
        
        # Draw object circles
        for obj in objects:
            cx, cy = obj['center']
            radius = obj['radius']
            circle = Circle((cx, cy), radius, color='red', fill=False, linewidth=2, linestyle='--')
            ax1.add_patch(circle)
            object_circles.append(circle)
            
            # Add label
            label = f"Obj"
            text = ax1.text(cx, cy, label, color='red', fontsize=8, 
                          ha='center', va='center', fontweight='bold')
            object_circles.append(text)
        
        # Update object list
        if objects:
            info_lines = [
                "DETECTED OBJECTS",
                "=" * 50,
                f"Total: {len(objects)} objects",
                "",
            ]
            
            for i, obj in enumerate(sorted(objects, key=lambda o: o['distance']), 1):
                info_lines.append(
                    f"Object {i}:\n"
                    f"  Position: X={obj['center'][0]:.1f}cm, Y={obj['center'][1]:.1f}cm\n"
                    f"            (X=front/back, Y=left/right)\n"
                    f"  Distance: {obj['distance']:.1f} cm\n"
                    f"  Angle:    {obj['angle']:.1f}° (0°=front, 90°=left)\n"
                    f"  Size:     {obj['radius']*2:.1f} cm diameter\n"
                    f"  Points:   {obj['points']}\n"
                )
            
            object_text.set_text('\n'.join(info_lines))
        else:
            object_text.set_text("DETECTED OBJECTS\n" + "="*50 + "\nNo objects detected")
    
    return scatter, object_text

# ----------------------------
# Main
# ----------------------------
if __name__ == "__main__":
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        print(f"Connected to LiDAR on {PORT}")
        print("Object detection running... Close window to stop.")
        print(f"\nDetection parameters:")
        print(f"  Update rate: ~{1000/UPDATE_INTERVAL:.0f} Hz")
        print(f"  Field of view: {FRONT_CONE_ANGLE}° (front cone)")
        print(f"  Cluster distance threshold: {CLUSTER_DISTANCE_THRESHOLD}cm")
        print(f"  Min cluster size: {MIN_CLUSTER_SIZE} points")
        print(f"  Max cluster size: {MAX_CLUSTER_SIZE} points")
        print()
        
        ani = FuncAnimation(fig, update, interval=UPDATE_INTERVAL, blit=False, cache_frame_data=False)
        plt.tight_layout()
        
        # Add timestamp display
        import time as time_module
        fig.suptitle(f'LiDAR Object Detection - Data timeout: {LIDAR_DATA_TIMEOUT}s', fontsize=10)
        
        plt.show()
        
    except KeyboardInterrupt:
        print("\nStopping...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'ser' in locals():
            ser.close()
        print("LiDAR disconnected")
