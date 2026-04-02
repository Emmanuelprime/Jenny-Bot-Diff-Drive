import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle
from collections import deque
import math
import time
import threading
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

# LiDAR angle calibration
LIDAR_ANGLE_OFFSET = 60  # degrees - offset to align LiDAR front with robot front

# Object detection parameters - DBSCAN clustering
MAX_DETECTION_DISTANCE = 100  # cm - only process objects within this range (saves CPU)
DBSCAN_EPS = 25  # cm - neighborhood distance (increased to merge object fragments)
DBSCAN_MIN_SAMPLES = 4  # minimum points for a cluster core (increased to filter noise)
MAX_CLUSTER_SIZE = 100  # maximum points in a cluster (filter out walls)

# Field of view - FULL 360° SCAN
USE_FULL_SCAN = True  # Process full 360° instead of just front cone

# Robot coordinate system:
#     +Y (90°, Left)
#      |
#      |
#      +---- +X (0°, Front)
#     /
#    / Robot

# Store recent points
point_buffer = deque(maxlen=MAX_POINTS)  # Thread-safe deque
# Note: Timestamp filtering removed from this visualization script
# because it processes buffered data at 30ms intervals. All points
# in the buffer are recent enough for display and detection.

# Thread control
stop_reading = False  # Signal to stop background thread

# Object tracking
tracked_objects = {}  # {object_id: {'center', 'radius', 'points', 'distance', 'angle', 'age', 'last_seen'}}
next_object_id = 1
OBJECT_MATCH_THRESHOLD = 30  # cm - max distance to match objects between frames
MAX_OBJECT_AGE = 5  # frames - remove objects not seen for this many frames
RECLUSTER_INTERVAL = 10  # frames - full re-cluster every N frames to avoid drift

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
            angle = (start_angle + i * angle_step + LIDAR_ANGLE_OFFSET) % 360
            
            if 2 < distance_cm < MAX_DISTANCE:  # 2cm to MAX_DISTANCE
                points.append((angle, distance_cm, quality))
    
    return start_angle, points

# ----------------------------
# Background LiDAR Reader Thread
# ----------------------------
def lidar_reader_thread(ser):
    """
    Background thread that continuously reads LiDAR data.
    Runs independently from animation loop for smooth visualization.
    """
    global stop_reading
    
    print("[Thread] LiDAR reader started")
    
    while not stop_reading:
        try:
            current_time = time.time()
            
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
        
        except Exception as e:
            if not stop_reading:  # Don't print errors during shutdown
                print(f"[Thread] Error reading LiDAR: {e}")
            time.sleep(0.01)
    
    print("[Thread] LiDAR reader stopped")

# ----------------------------
# DBSCAN Clustering Algorithm
# ----------------------------
def dbscan_clustering(points_xy, eps=DBSCAN_EPS, min_samples=DBSCAN_MIN_SAMPLES):
    """
    DBSCAN: Density-Based Spatial Clustering of Applications with Noise
    
    Args:
        points_xy: List of (x, y) coordinates
        eps: Maximum distance between two points to be neighbors (cm)
        min_samples: Minimum points in neighborhood to form a cluster core
    
    Returns:
        labels: Array where labels[i] is cluster ID for point i (-1 = noise)
    """
    n = len(points_xy)
    labels = [-1] * n  # -1 = unvisited/noise
    cluster_id = 0
    
    def get_neighbors(point_idx):
        """Find all points within eps distance of point_idx"""
        neighbors = []
        px, py = points_xy[point_idx]
        for i, (x, y) in enumerate(points_xy):
            dist = math.sqrt((x - px)**2 + (y - py)**2)
            if dist <= eps:
                neighbors.append(i)
        return neighbors
    
    def expand_cluster(point_idx, neighbors, cluster_id):
        """Expand cluster from a core point"""
        labels[point_idx] = cluster_id
        
        i = 0
        while i < len(neighbors):
            neighbor_idx = neighbors[i]
            
            # If unvisited, mark as part of cluster
            if labels[neighbor_idx] == -1:
                labels[neighbor_idx] = cluster_id
                
                # If this point is also a core point, add its neighbors
                neighbor_neighbors = get_neighbors(neighbor_idx)
                if len(neighbor_neighbors) >= min_samples:
                    neighbors.extend([n for n in neighbor_neighbors if n not in neighbors])
            
            i += 1
    
    # Main DBSCAN loop
    for point_idx in range(n):
        if labels[point_idx] != -1:  # Already processed
            continue
        
        neighbors = get_neighbors(point_idx)
        
        if len(neighbors) < min_samples:
            # Not enough neighbors - mark as noise (stays -1)
            continue
        
        # Start new cluster from this core point
        expand_cluster(point_idx, neighbors, cluster_id)
        cluster_id += 1
    
    return labels

# ----------------------------
# Object Tracking Functions
# ----------------------------
def match_objects(new_objects, tracked_objects, threshold=OBJECT_MATCH_THRESHOLD):
    """
    Match newly detected objects to existing tracked objects using nearest neighbor.
    
    Returns:
        matches: dict {new_idx: object_id}
        unmatched_new: list of indices of new objects that didn't match
    """
    matches = {}
    unmatched_new = list(range(len(new_objects)))
    
    if not tracked_objects or not new_objects:
        return matches, unmatched_new
    
    # For each tracked object, find closest new detection
    for obj_id, tracked_obj in tracked_objects.items():
        if not new_objects:
            break
            
        tracked_center = tracked_obj['center']
        
        # Find closest new object
        min_dist = float('inf')
        closest_idx = None
        
        for i in unmatched_new:
            new_center = new_objects[i]['center']
            dist = math.sqrt((new_center[0] - tracked_center[0])**2 + 
                           (new_center[1] - tracked_center[1])**2)
            
            if dist < min_dist and dist < threshold:
                min_dist = dist
                closest_idx = i
        
        # If found a match, assign it
        if closest_idx is not None:
            matches[closest_idx] = obj_id
            unmatched_new.remove(closest_idx)
    
    return matches, unmatched_new

def update_tracked_objects(new_objects, frame_number):
    """
    Update tracked objects with new detections.
    Handles matching, adding new objects, and removing stale ones.
    """
    global tracked_objects, next_object_id
    
    # Match new detections to existing tracked objects
    matches, unmatched_new = match_objects(new_objects, tracked_objects)
    
    # Update matched objects
    matched_ids = set()
    for new_idx, obj_id in matches.items():
        tracked_objects[obj_id] = new_objects[new_idx]
        tracked_objects[obj_id]['age'] = tracked_objects[obj_id].get('age', 0) + 1
        tracked_objects[obj_id]['last_seen'] = frame_number
        tracked_objects[obj_id]['id'] = obj_id  # Keep ID
        matched_ids.add(obj_id)
    
    # Add new unmatched objects
    for new_idx in unmatched_new:
        tracked_objects[next_object_id] = new_objects[new_idx]
        tracked_objects[next_object_id]['age'] = 1
        tracked_objects[next_object_id]['last_seen'] = frame_number
        tracked_objects[next_object_id]['id'] = next_object_id
        next_object_id += 1
    
    # Remove stale objects (not seen recently)
    stale_ids = [obj_id for obj_id, obj in tracked_objects.items() 
                 if frame_number - obj['last_seen'] > MAX_OBJECT_AGE]
    for obj_id in stale_ids:
        del tracked_objects[obj_id]
    
    return list(tracked_objects.values())

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
# Cluster points into objects using DBSCAN (with tracking)
# ----------------------------
frame_counter = 0

def detect_objects():
    global frame_counter, tracked_objects
    
    frame_counter += 1
    
    if len(point_buffer) == 0:
        return list(tracked_objects.values())
    
    # Only do full DBSCAN clustering periodically or on first frame
    should_recluster = (frame_counter % RECLUSTER_INTERVAL == 1) or (frame_counter == 1)
    
    if not should_recluster:
        # Just return existing tracked objects (updated positions from previous frame)
        return list(tracked_objects.values())
    
    # Full DBSCAN clustering
    all_points = list(point_buffer)
    
    if len(all_points) == 0:
        return list(tracked_objects.values())
    
    # Convert polar to Cartesian and filter by distance from robot
    points_xy = []
    for angle, distance, timestamp in all_points:
        x, y = polar_to_cartesian(angle, distance)
        # Calculate distance from robot (origin)
        dist_from_robot = math.sqrt(x**2 + y**2)
        # Only process objects within detection range
        if dist_from_robot <= MAX_DETECTION_DISTANCE:
            points_xy.append((x, y))
    
    if len(points_xy) == 0:
        return list(tracked_objects.values())
    
    # Apply DBSCAN clustering
    labels = dbscan_clustering(points_xy, eps=DBSCAN_EPS, min_samples=DBSCAN_MIN_SAMPLES)
    
    # Group points by cluster
    clusters_dict = {}
    for i, label in enumerate(labels):
        if label == -1:  # Skip noise points
            continue
        if label not in clusters_dict:
            clusters_dict[label] = []
        clusters_dict[label].append(points_xy[i])
    
    # Calculate object properties (new detections)
    new_objects = []
    for cluster_id, cluster in clusters_dict.items():
        # Filter out too-large clusters (walls/background)
        if len(cluster) > MAX_CLUSTER_SIZE:
            continue
        
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
        
        new_objects.append({
            'center': (center_x, center_y),
            'radius': max_radius,
            'points': len(cluster),
            'distance': distance_from_robot,
            'angle': angle_from_robot
        })
    
    # Update tracked objects with new detections
    return update_tracked_objects(new_objects, frame_counter)

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
ax1.set_title('Object Detection - Full 360° Scan (Front=+X, Left=+Y)')

# Draw robot at center (15cm radius)
robot_circle = Circle((0, 0), 15, color='red', fill=True, alpha=0.5)
ax1.add_patch(robot_circle)
# Arrow pointing in +X direction (forward)
ax1.arrow(0, 0, 30, 0, head_width=10, head_length=10, fc='red', ec='red')
# Label for orientation
ax1.text(35, 0, 'FRONT', fontsize=8, color='red', fontweight='bold')

# Add legend for distance colors
from matplotlib.lines import Line2D
legend_elements = [
    Line2D([0], [0], marker='o', color='w', markerfacecolor='red', markersize=10, label='<30cm (Very Close)'),
    Line2D([0], [0], marker='o', color='w', markerfacecolor='orange', markersize=10, label='30-60cm (Close)'),
    Line2D([0], [0], marker='o', color='w', markerfacecolor='yellow', markersize=10, label='60-100cm (Medium)'),
    Line2D([0], [0], color='orange', linestyle='--', linewidth=2, label=f'{MAX_DETECTION_DISTANCE}cm Detection Limit'),
]
ax1.legend(handles=legend_elements, loc='upper left', fontsize=8)

# Draw detection range circle (100cm limit for obstacle processing)
detection_circle = Circle((0, 0), MAX_DETECTION_DISTANCE, color='orange', 
                         fill=False, linewidth=2, linestyle='--', alpha=0.5, 
                         label=f'{MAX_DETECTION_DISTANCE}cm Range')
ax1.add_patch(detection_circle)

scatter = ax1.scatter([], [], c='blue', s=5, alpha=0.4, label='LiDAR points')
object_circles = []
warning_text = ax1.text(0, MAX_DISTANCE * 0.9, '', fontsize=12, color='red', 
                       fontweight='bold', ha='center', 
                       bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.8))

# Right: Object list
ax2.axis('off')
ax2.set_xlim(0, 1)
ax2.set_ylim(0, 1)
object_text = ax2.text(0.05, 0.95, '', verticalalignment='top', fontfamily='monospace', fontsize=10)

# ----------------------------
# Animation update function
# ----------------------------
def update(frame):
    # No blocking serial read here - background thread populates buffer
    
    # Clear previous object circles
    for circle in object_circles:
        circle.remove()
    object_circles.clear()
    
    if len(point_buffer) > 0:
        # Plot all points from 360° scan
        all_points = list(point_buffer)
        
        # Convert all points to Cartesian for display
        x_coords = []
        y_coords = []
        
        for angle_deg, distance, timestamp in all_points:
            x, y = polar_to_cartesian(angle_deg, distance)
            x_coords.append(x)
            y_coords.append(y)
        
        scatter.set_offsets(np.c_[x_coords, y_coords])
        
        # Detect objects (filters by distance internally)
        objects = detect_objects()
        
        # Draw object circles with distance-based coloring
        close_objects = []
        for i, obj in enumerate(objects, 1):
            cx, cy = obj['center']
            radius = obj['radius']
            dist = obj['distance']
            
            # Color code by distance (optimized for 0-100cm range):
            if dist < 30:
                color = 'red'
                linewidth = 3
                close_objects.append(obj)
            elif dist < 60:
                color = 'orange'
                linewidth = 2.5
                close_objects.append(obj)
            elif dist < 100:
                color = 'yellow'
                linewidth = 2
                close_objects.append(obj)
            else:
                # This shouldn't happen with MAX_DETECTION_DISTANCE filter
                color = 'green'
                linewidth = 1.5
            
            # Draw object circle
            circle = Circle((cx, cy), radius, color=color, fill=False, linewidth=linewidth, linestyle='-')
            ax1.add_patch(circle)
            object_circles.append(circle)
            
            # Add detailed label with persistent ID
            obj_id = obj.get('id', i)
            label = f"ID:{obj_id}\n{dist:.0f}cm"
            text = ax1.text(cx, cy, label, color=color, fontsize=9, 
                          ha='center', va='center', fontweight='bold',
                          bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.7))
            object_circles.append(text)
        
        # Update warning text for close objects (<60cm)
        if close_objects:
            closest = min(close_objects, key=lambda o: o['distance'])
            warning_text.set_text(f"⚠️ {len(close_objects)} OBSTACLE(S) <60cm | CLOSEST: {closest['distance']:.0f}cm")
            warning_text.set_visible(True)
        else:
            warning_text.set_visible(False)
        
        # Update object list
        if objects:
            # Sort by distance
            sorted_objects = sorted(objects, key=lambda o: o['distance'])
            
            info_lines = [
                "TRACKED OBJECTS (360° SCAN)",
                "=" * 50,
                f"Total: {len(objects)} objects",
                f"Frame: {frame_counter} (Re-cluster: {frame_counter % RECLUSTER_INTERVAL == 1})\n",
            ]
            
            for i, obj in enumerate(sorted_objects, 1):
                dist = obj['distance']
                angle = obj['angle']
                obj_id = obj.get('id', '?')
                age = obj.get('age', 0)
                
                # Status indicator (updated for 0-100cm range)
                if dist < 30:
                    status = "🔴 VERY CLOSE"
                elif dist < 60:
                    status = "🟠 CLOSE"
                elif dist < 100:
                    status = "🟡 MEDIUM"
                else:
                    status = "🟢 FAR"  # Shouldn't happen with filter
                
                # Direction (for 360° coverage)
                if -15 <= angle <= 15:
                    direction = "FRONT"
                elif 15 < angle <= 75:
                    direction = "FRONT-LEFT"
                elif 75 < angle <= 105:
                    direction = "LEFT"
                elif 105 < angle <= 165:
                    direction = "REAR-LEFT"
                elif 165 < angle or angle < -165:
                    direction = "REAR"
                elif -165 <= angle < -105:
                    direction = "REAR-RIGHT"
                elif -105 <= angle < -75:
                    direction = "RIGHT"
                else:  # -75 to -15
                    direction = "FRONT-RIGHT"
                
                info_lines.append(
                    f"ID:{obj_id} {status} (age:{age})\n"
                    f"  Dist: {dist:.0f}cm | Angle: {angle:+.1f}°\n"
                    f"  Direction: {direction}\n"
                    f"  Position: X={obj['center'][0]:.0f} Y={obj['center'][1]:.0f}cm\n"
                    f"  Size: {obj['radius']*2:.0f}cm ({obj['points']} pts)\n"
                )
            
            object_text.set_text('\n'.join(info_lines))
        else:
            object_text.set_text(f"TRACKED OBJECTS (360° SCAN)\n" + "="*50 + f"\nFrame: {frame_counter}\n\nNo objects detected\n\nClear area ✓")
            warning_text.set_visible(False)
    
    return scatter, object_text, warning_text

# ----------------------------
# Main
# ----------------------------
if __name__ == "__main__":
    ser = None
    reader_thread = None
    
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        print(f"Connected to LiDAR on {PORT}")
        print("Object detection running... Close window to stop.")
        print(f"\nDetection parameters:")
        print(f"  Update rate: ~{1000/UPDATE_INTERVAL:.0f} Hz")
        print(f"  Field of view: 360° (full scan)")
        print(f"  Detection range: 0-{MAX_DETECTION_DISTANCE}cm (ignores far objects)")
        print(f"  Clustering: DBSCAN with object tracking")
        print(f"    - eps (neighborhood): {DBSCAN_EPS}cm")
        print(f"    - min_samples: {DBSCAN_MIN_SAMPLES} points")
        print(f"    - max cluster size: {MAX_CLUSTER_SIZE} points")
        print(f"  Tracking:")
        print(f"    - Re-cluster interval: every {RECLUSTER_INTERVAL} frames")
        print(f"    - Match threshold: {OBJECT_MATCH_THRESHOLD}cm")
        print(f"    - Max object age: {MAX_OBJECT_AGE} frames")
        print(f"  Threading: Background LiDAR reader (non-blocking)")
        print()
        
        # Start background LiDAR reader thread
        reader_thread = threading.Thread(target=lidar_reader_thread, args=(ser,), daemon=True)
        reader_thread.start()
        
        # Give thread time to start collecting data
        time.sleep(0.5)
        
        ani = FuncAnimation(fig, update, interval=UPDATE_INTERVAL, blit=False, cache_frame_data=False)
        plt.tight_layout()
        
        # Add title
        fig.suptitle(f'LiDAR Object Detection - Full 360° Scan with Tracking | Re-cluster every {RECLUSTER_INTERVAL} frames', 
                     fontsize=12, fontweight='bold')
        
        plt.show()
        
    except KeyboardInterrupt:
        print("\nStopping...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Clean shutdown
        stop_reading = True
        if reader_thread and reader_thread.is_alive():
            reader_thread.join(timeout=2.0)
        if ser:
            ser.close()
        print("LiDAR disconnected")
