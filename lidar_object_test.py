import serial
import math
import time
from collections import deque

# ----------------------------
# Configuration
# ----------------------------
PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
DISTANCE_SCALE = 0.001  # mm to meters
MAX_POINTS = 1000

# Object detection parameters
CLUSTER_DISTANCE_THRESHOLD = 0.15  # meters
MIN_CLUSTER_SIZE = 3
MAX_CLUSTER_SIZE = 100
DETECTION_RANGE = (0.1, 3.0)  # min and max distance in meters

# ----------------------------
# Parse LiDAR packet (X3 protocol)
# ----------------------------
def parse_packet(packet_bytes):
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
            
            if DETECTION_RANGE[0] < distance < DETECTION_RANGE[1]:
                points.append((angle, distance, quality))
    
    return start_angle, points

# ----------------------------
# Collect scan data
# ----------------------------
def collect_scan(ser, num_packets=50):
    point_buffer = []
    packets_read = 0
    
    while packets_read < num_packets:
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
                    point_buffer.append((angle, distance))
            
            packets_read += 1
    
    return point_buffer

# ----------------------------
# Convert polar to Cartesian
# ----------------------------
def polar_to_cartesian(angle_deg, distance):
    angle_rad = math.radians(angle_deg)
    x = distance * math.sin(angle_rad)
    y = distance * math.cos(angle_rad)
    return x, y

# ----------------------------
# Detect objects
# ----------------------------
def detect_objects(point_buffer):
    if len(point_buffer) == 0:
        return []
    
    # Convert to Cartesian
    points_xy = [polar_to_cartesian(angle, dist) for angle, dist in point_buffer]
    
    # Clustering
    clusters = []
    visited = [False] * len(points_xy)
    
    for i, (x1, y1) in enumerate(points_xy):
        if visited[i]:
            continue
        
        cluster = [(x1, y1)]
        visited[i] = True
        
        for j, (x2, y2) in enumerate(points_xy):
            if visited[j]:
                continue
            
            for cx, cy in cluster:
                dist = math.sqrt((x2 - cx)**2 + (y2 - cy)**2)
                if dist < CLUSTER_DISTANCE_THRESHOLD:
                    cluster.append((x2, y2))
                    visited[j] = True
                    break
        
        if MIN_CLUSTER_SIZE <= len(cluster) <= MAX_CLUSTER_SIZE:
            clusters.append(cluster)
    
    # Calculate object properties
    objects = []
    for cluster in clusters:
        xs = [x for x, y in cluster]
        ys = [y for x, y in cluster]
        
        center_x = sum(xs) / len(xs)
        center_y = sum(ys) / len(ys)
        
        max_radius = max(math.sqrt((x - center_x)**2 + (y - center_y)**2) for x, y in cluster)
        
        distance_from_robot = math.sqrt(center_x**2 + center_y**2)
        angle_from_robot = math.degrees(math.atan2(center_x, center_y))
        if angle_from_robot < 0:
            angle_from_robot += 360
        
        objects.append({
            'center': (center_x, center_y),
            'radius': max_radius,
            'points': len(cluster),
            'distance': distance_from_robot,
            'angle': angle_from_robot
        })
    
    return sorted(objects, key=lambda o: o['distance'])

# ----------------------------
# Get obstacles in specific sector
# ----------------------------
def get_obstacles_in_sector(objects, center_angle, cone_width=30, max_distance=1.0):
    """
    Filter objects within a cone/sector
    center_angle: direction in degrees (0 = forward)
    cone_width: width in degrees
    max_distance: maximum distance to consider
    """
    obstacles = []
    for obj in objects:
        if obj['distance'] > max_distance:
            continue
        
        angle_diff = abs((obj['angle'] - center_angle + 180) % 360 - 180)
        if angle_diff < cone_width / 2:
            obstacles.append(obj)
    
    return obstacles

# ----------------------------
# Main detection loop
# ----------------------------
def main():
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    print(f"Connected to LiDAR on {PORT}\n")
    print(f"Object Detection Parameters:")
    print(f"  Detection range: {DETECTION_RANGE[0]}-{DETECTION_RANGE[1]}m")
    print(f"  Cluster threshold: {CLUSTER_DISTANCE_THRESHOLD}m")
    print(f"  Min/Max points: {MIN_CLUSTER_SIZE}/{MAX_CLUSTER_SIZE}")
    print("\n" + "="*70)
    print()
    
    try:
        scan_count = 0
        while True:
            scan_count += 1
            start_time = time.time()
            
            # Collect scan
            point_buffer = collect_scan(ser, num_packets=50)
            
            # Detect objects
            objects = detect_objects(point_buffer)
            
            # Display results
            print(f"\n{'='*70}")
            print(f"SCAN #{scan_count} - {len(point_buffer)} points, {len(objects)} objects detected")
            print(f"{'='*70}")
            
            if objects:
                for i, obj in enumerate(objects, 1):
                    print(f"\nObject {i}:")
                    print(f"  Location:  ({obj['center'][0]:6.2f}, {obj['center'][1]:6.2f}) m")
                    print(f"  Distance:  {obj['distance']:5.2f} m")
                    print(f"  Angle:     {obj['angle']:6.1f}°")
                    print(f"  Size:      {obj['radius']*2:5.2f} m diameter")
                    print(f"  Points:    {obj['points']}")
                
                # Check front sector
                front_obstacles = get_obstacles_in_sector(objects, center_angle=0, cone_width=60, max_distance=1.0)
                if front_obstacles:
                    print(f"\n⚠️  WARNING: {len(front_obstacles)} obstacle(s) in front (60° cone, <1m)")
                    for obs in front_obstacles:
                        print(f"    - Distance: {obs['distance']:.2f}m at {obs['angle']:.1f}°")
            else:
                print("\nNo objects detected")
            
            # Timing info
            scan_time = time.time() - start_time
            print(f"\nScan time: {scan_time:.2f}s")
            
            time.sleep(0.5)  # Brief pause between scans
            
    except KeyboardInterrupt:
        print("\n\nStopped by user")
    finally:
        ser.close()
        print("LiDAR disconnected")

if __name__ == "__main__":
    main()
