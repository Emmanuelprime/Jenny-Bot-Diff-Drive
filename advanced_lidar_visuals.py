import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle, Circle
from collections import deque
import math
from scipy import ndimage
from sklearn.cluster import DBSCAN
import warnings
warnings.filterwarnings('ignore')

# ----------------------------
# Configuration
# ----------------------------
PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
PACKET_HEADER = 0xAA
DISTANCE_SCALE = 0.001  # mm to meters
POINTS_PER_PACKET = 12
MAX_POINTS = 2000  # Store multiple full rotations
MAX_DISTANCE = 6.0  # meters

# Filtering parameters
MIN_DISTANCE = 0.15  # Minimum valid distance (filter out noise close to sensor)
MAX_DISTANCE = 6.0   # Maximum valid distance
INTENSITY_THRESHOLD = 10  # Minimum signal quality

# Wall detection parameters
WALL_SEGMENT_LENGTH = 0.3  # Minimum wall segment length (meters)
WALL_GAP_TOLERANCE = 0.15  # Maximum gap between points on same wall (meters)
ANGLE_TOLERANCE = 5.0  # Degrees tolerance for wall alignment

# Obstacle detection parameters
CLUSTER_EPS = 0.2  # DBSCAN epsilon (meters) - maximum distance between points in same cluster
MIN_CLUSTER_SIZE = 5  # Minimum points to form an obstacle
OBSTACLE_MIN_SIZE = 0.1  # Minimum obstacle size (meters)

# Store recent points and processed data
point_buffer = deque(maxlen=MAX_POINTS)
filtered_buffer = deque(maxlen=MAX_POINTS)
wall_segments = []
obstacles = []

# ----------------------------
# Advanced Filtering Functions
# ----------------------------
def median_filter(distances, window_size=5):
    """Apply median filter to smooth distance measurements"""
    if len(distances) < window_size:
        return distances
    return ndimage.median_filter(distances, size=window_size)

def remove_outliers(points, std_threshold=2.0):
    """Remove statistical outliers based on local neighborhood"""
    if len(points) < 10:
        return points
    
    distances = np.array([d for _, d, _ in points])
    angles = np.array([a for a, _, _ in points])
    
    # Use rolling window statistics
    filtered_points = []
    window = min(20, len(points))
    
    for i in range(len(points)):
        start_idx = max(0, i - window//2)
        end_idx = min(len(points), i + window//2)
        
        local_distances = distances[start_idx:end_idx]
        mean_dist = np.mean(local_distances)
        std_dist = np.std(local_distances)
        
        if abs(distances[i] - mean_dist) < std_threshold * std_dist:
            filtered_points.append(points[i])
    
    return filtered_points

def temporal_filter(new_points, alpha=0.3):
    """Apply temporal filtering for smoother transitions"""
    global point_buffer
    
    if len(point_buffer) == 0:
        return new_points
    
    # Convert to dictionary for easier lookup
    old_dict = {round(angle, 1): (dist, qual) for angle, dist, qual in point_buffer}
    filtered = []
    
    for angle, dist, qual in new_points:
        angle_key = round(angle, 1)
        if angle_key in old_dict:
            old_dist = old_dict[angle_key][0]
            # Exponential moving average
            filtered_dist = alpha * dist + (1 - alpha) * old_dist
            filtered.append((angle, filtered_dist, qual))
        else:
            filtered.append((angle, dist, qual))
    
    return filtered

# ----------------------------
# Wall Detection
# ----------------------------
def detect_walls(points):
    """Detect straight wall segments using RANSAC-like approach"""
    if len(points) < 10:
        return []
    
    walls = []
    points_used = set()
    
    # Convert to Cartesian coordinates
    cartesian_points = []
    for angle, dist, _ in points:
        x = dist * np.cos(np.radians(angle))
        y = dist * np.sin(np.radians(angle))
        cartesian_points.append((x, y, angle, dist))
    
    cartesian_points = np.array(cartesian_points)
    
    # Simple line detection using sequential points
    i = 0
    while i < len(cartesian_points) - MIN_CLUSTER_SIZE:
        if i in points_used:
            i += 1
            continue
        
        segment_points = []
        current_line = []
        
        # Start a potential wall segment
        for j in range(i, min(i + 30, len(cartesian_points))):
            if j in points_used:
                break
            
            if len(current_line) == 0:
                current_line.append(j)
                continue
            
            # Calculate distance between consecutive points
            p1 = cartesian_points[current_line[-1]]
            p2 = cartesian_points[j]
            point_dist = np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
            
            # If points are close, add to current line
            if point_dist < WALL_GAP_TOLERANCE:
                current_line.append(j)
            else:
                # Check if we have a valid wall segment
                if len(current_line) >= 10:
                    # Calculate line parameters
                    p_start = cartesian_points[current_line[0]]
                    p_end = cartesian_points[current_line[-1]]
                    segment_length = np.sqrt((p_end[0] - p_start[0])**2 + 
                                           (p_end[1] - p_start[1])**2)
                    
                    if segment_length > WALL_SEGMENT_LENGTH:
                        walls.append({
                            'start': (p_start[0], p_start[1]),
                            'end': (p_end[0], p_end[1]),
                            'length': segment_length,
                            'angle': np.degrees(np.arctan2(p_end[1] - p_start[1], 
                                                          p_end[0] - p_start[0]))
                        })
                        for idx in current_line:
                            points_used.add(idx)
                break
        
        i += 1
    
    return walls

# ----------------------------
# Obstacle Detection
# ----------------------------
def detect_obstacles(points):
    """Detect obstacles using DBSCAN clustering"""
    if len(points) < MIN_CLUSTER_SIZE:
        return []
    
    # Convert to Cartesian coordinates
    cartesian_coords = []
    for angle, dist, quality in points:
        x = dist * np.cos(np.radians(angle))
        y = dist * np.sin(np.radians(angle))
        cartesian_coords.append([x, y])
    
    cartesian_coords = np.array(cartesian_coords)
    
    # Apply DBSCAN clustering
    clustering = DBSCAN(eps=CLUSTER_EPS, min_samples=MIN_CLUSTER_SIZE).fit(cartesian_coords)
    labels = clustering.labels_
    
    obstacles = []
    for label in set(labels):
        if label == -1:  # Noise points
            continue
        
        # Get points in this cluster
        cluster_points = cartesian_coords[labels == label]
        
        # Calculate cluster properties
        center = np.mean(cluster_points, axis=0)
        min_coords = np.min(cluster_points, axis=0)
        max_coords = np.max(cluster_points, axis=0)
        size = np.sqrt((max_coords[0] - min_coords[0])**2 + 
                      (max_coords[1] - min_coords[1])**2)
        
        # Calculate distance from sensor
        distance = np.sqrt(center[0]**2 + center[1]**2)
        angle = np.degrees(np.arctan2(center[1], center[0]))
        
        if size > OBSTACLE_MIN_SIZE:
            obstacles.append({
                'center': (center[0], center[1]),
                'size': size,
                'distance': distance,
                'angle': angle,
                'num_points': len(cluster_points),
                'bbox': (min_coords, max_coords)
            })
    
    return obstacles

# ----------------------------
# Parse LiDAR packet (X3 protocol)
# ----------------------------
def parse_packet(packet_bytes):
    """Parse LiDAR packet with quality filtering"""
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
            distance = dist_raw * 0.25 * DISTANCE_SCALE
            angle = (start_angle + i * angle_step) % 360
            
            # Apply basic filtering
            if MIN_DISTANCE < distance < MAX_DISTANCE and quality > INTENSITY_THRESHOLD:
                points.append((angle, distance, quality))
    
    return start_angle, points

# ----------------------------
# Read LiDAR data with buffering
# ----------------------------
def read_lidar_data(ser):
    """Read multiple packets for complete scan"""
    packets_read = 0
    new_points = []
    
    while packets_read < 4:  # Read multiple packets per update
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
                new_points.extend(points)
            
            packets_read += 1
    
    if new_points:
        # Apply filtering pipeline
        filtered = remove_outliers(new_points)
        filtered = temporal_filter(filtered)
        
        for point in filtered:
            point_buffer.append(point)

# ----------------------------
# Advanced Visualization Setup
# ----------------------------
fig = plt.figure(figsize=(16, 8))

# Polar plot for raw data
ax1 = fig.add_subplot(121, projection='polar')
ax1.set_ylim(0, MAX_DISTANCE)
ax1.set_theta_zero_location('N')
ax1.set_theta_direction(-1)
ax1.grid(True, alpha=0.3)
ax1.set_title('LiDAR Raw Scan', pad=20, fontsize=14, fontweight='bold')

# Cartesian plot for processed data with walls/obstacles
ax2 = fig.add_subplot(122)
ax2.set_xlim(-MAX_DISTANCE, MAX_DISTANCE)
ax2.set_ylim(-MAX_DISTANCE, MAX_DISTANCE)
ax2.set_aspect('equal')
ax2.grid(True, alpha=0.3)
ax2.set_xlabel('X (meters)', fontsize=12)
ax2.set_ylabel('Y (meters)', fontsize=12)
ax2.set_title('Processed Scan with Walls & Obstacles', pad=20, fontsize=14, fontweight='bold')

# Add sensor position marker
sensor_marker = Circle((0, 0), 0.05, color='red', alpha=0.8, label='LiDAR')
ax2.add_patch(sensor_marker)

# Initialize visualization elements
scatter_raw = ax1.scatter([], [], c='blue', s=2, alpha=0.6, label='Raw Points')
scatter_filtered = ax2.scatter([], [], c='green', s=3, alpha=0.5, label='Filtered Points')
wall_lines = []
obstacle_patches = []

# Text annotations
info_text = ax2.text(0.02, 0.98, '', transform=ax2.transAxes, 
                     verticalalignment='top', fontsize=10, 
                     bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

# ----------------------------
# Animation Update Function
# ----------------------------
def update(frame):
    global wall_segments, obstacles, wall_lines, obstacle_patches
    
    # Read new LiDAR data
    read_lidar_data(ser)
    
    # Clear previous dynamic elements
    for line in wall_lines:
        line.remove()
    wall_lines.clear()
    
    for patch in obstacle_patches:
        patch.remove()
    obstacle_patches.clear()
    
    if len(point_buffer) > 0:
        # Convert to numpy arrays for processing
        points_array = np.array(list(point_buffer))
        angles = points_array[:, 0]
        distances = points_array[:, 1]
        qualities = points_array[:, 2] if points_array.shape[1] > 2 else np.ones_like(distances)
        
        # Update raw data visualization
        angles_rad = np.radians(angles)
        scatter_raw.set_offsets(np.c_[angles_rad, distances])
        colors = plt.cm.viridis(distances / MAX_DISTANCE)
        scatter_raw.set_color(colors)
        
        # Process for walls and obstacles
        points_list = [(angles[i], distances[i], qualities[i]) for i in range(len(angles))]
        wall_segments = detect_walls(points_list)
        obstacles = detect_obstacles(points_list)
        
        # Update Cartesian visualization
        x_coords = distances * np.cos(angles_rad)
        y_coords = distances * np.sin(angles_rad)
        scatter_filtered.set_offsets(np.c_[x_coords, y_coords])
        scatter_filtered.set_color(colors)
        
        # Draw walls
        for wall in wall_segments:
            line = ax2.plot([wall['start'][0], wall['end'][0]], 
                           [wall['start'][1], wall['end'][1]], 
                           'r-', linewidth=3, alpha=0.8)
            wall_lines.extend(line)
        
        # Draw obstacles
        for obs in obstacles:
            center = obs['center']
            size = obs['size']
            
            # Draw obstacle circle
            circle = Circle(center, size/2, fill=False, edgecolor='orange', 
                          linewidth=2, alpha=0.7)
            ax2.add_patch(circle)
            obstacle_patches.append(circle)
            
            # Add distance label
            if obs['distance'] < MAX_DISTANCE * 0.8:
                label = ax2.annotate(f"{obs['distance']:.1f}m", 
                                   xy=center, xytext=(5, 5), 
                                   textcoords='offset points',
                                   fontsize=8, color='orange',
                                   bbox=dict(boxstyle='round', facecolor='white', alpha=0.7))
                obstacle_patches.append(label)
        
        # Update info text
        info_text.set_text(f"Points: {len(points_array)}\n"
                          f"Walls: {len(wall_segments)}\n"
                          f"Obstacles: {len(obstacles)}\n"
                          f"Range: {distances.min():.2f}-{distances.max():.2f}m")
    
    # Update legend
    if len(wall_segments) > 0 or len(obstacles) > 0:
        legend_elements = [sensor_marker, 
                         plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='green', 
                                  markersize=5, label='Filtered Points'),
                         plt.Line2D([0], [0], color='red', linewidth=3, label='Walls'),
                         plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='orange', 
                                  markersize=8, label='Obstacles')]
        ax2.legend(handles=legend_elements, loc='upper right')
    
    return scatter_raw, scatter_filtered

# ----------------------------
# Main Execution
# ----------------------------
if __name__ == "__main__":
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        print(f"✓ Connected to LiDAR on {PORT}")
        print("→ Starting advanced visualization...")
        print("  • Green points: Filtered LiDAR data")
        print("  • Red lines: Detected walls")
        print("  • Orange circles: Detected obstacles")
        print("→ Close window to stop.")
        
        ani = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
        plt.tight_layout()
        plt.show()
        
    except KeyboardInterrupt:
        print("\n→ Stopping visualization...")
    except serial.SerialException as e:
        print(f"✗ Serial connection error: {e}")
        print("  Please check:")
        print("  - LiDAR is properly connected")
        print("  - Correct port permissions (try: sudo chmod 666 /dev/ttyUSB0)")
        print("  - Port is not in use by another program")
    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'ser' in locals():
            ser.close()
        print("✓ LiDAR disconnected")