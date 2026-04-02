import serial
import time
from collections import defaultdict

PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
DISTANCE_SCALE = 0.001

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
            distance_m = dist_raw * 0.25 * DISTANCE_SCALE
            distance_cm = distance_m * 100
            angle = (start_angle + i * angle_step) % 360
            
            # Only show valid detections (not noise)
            if 10 < distance_cm < 300 and quality > 10:  # Filter out quality=0 noise
                points.append((angle, distance_cm, quality))
    
    return start_angle, points

def group_angle(angle, bin_size=10):
    """Group angles into 10-degree bins for counting"""
    return int(angle / bin_size) * bin_size

print("="*70)
print("LIDAR FRONT ANGLE FINDER")
print("="*70)
print()
print("INSTRUCTIONS:")
print("1. Clear area around robot (remove close objects)")
print("2. Place ONE object at 40-60cm directly IN FRONT of robot")
print("3. Let it scan for 10 seconds")
print("4. The angle that appears MOST OFTEN is your front")
print()
print("="*70)
print()

ser = serial.Serial(PORT, BAUDRATE, timeout=1)
time.sleep(1)

# Accumulate detections by angle
angle_counts = defaultdict(int)
angle_distances = defaultdict(list)
scan_duration = 10  # seconds
start_time = time.time()

print(f"Scanning for {scan_duration} seconds...")
print("(Progress: ", end="", flush=True)

try:
    while time.time() - start_time < scan_duration:
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
                # Count detections in target range
                for angle, dist, qual in points:
                    if 30 < dist < 80:  # Target range for front object
                        angle_bin = group_angle(angle, 10)
                        angle_counts[angle_bin] += 1
                        angle_distances[angle_bin].append(dist)
        
        # Show progress
        elapsed = time.time() - start_time
        if int(elapsed) % 2 == 0 and int(elapsed * 10) % 10 == 0:
            print(".", end="", flush=True)
    
    print(" Done!)\n")
    print()
    print("="*70)
    print("RESULTS: Objects detected at 30-80cm range")
    print("="*70)
    print()
    
    # Sort by frequency
    sorted_angles = sorted(angle_counts.items(), key=lambda x: x[1], reverse=True)
    
    if sorted_angles:
        print(f"{'Angle Range':<15} {'Detections':<12} {'Avg Distance':<15}")
        print("-" * 45)
        
        for angle_bin, count in sorted_angles[:10]:  # Top 10
            avg_dist = sum(angle_distances[angle_bin]) / len(angle_distances[angle_bin])
            print(f"{angle_bin:3d}°-{angle_bin+9:3d}°     {count:6d}         {avg_dist:5.1f}cm")
        
        print()
        print("="*70)
        most_common_angle = sorted_angles[0][0]
        avg_distance = sum(angle_distances[most_common_angle]) / len(angle_distances[most_common_angle])
        
        print(f"MOST FREQUENT ANGLE: {most_common_angle}°-{most_common_angle+9}° ({sorted_angles[0][1]} detections)")
        print(f"Average distance: {avg_distance:.1f}cm")
        print()
        
        # Calculate offset needed to bring this angle to 0°
        offset = -most_common_angle
        if offset < -180:
            offset += 360
        elif offset > 180:
            offset -= 360
        
        print(f"Suggested LIDAR_ANGLE_OFFSET: {offset}°")
        print(f"(Add this offset to all angles to align LiDAR front with robot front)")
        print()
    else:
        print("No objects detected in 30-80cm range!")
        print("Make sure object is placed directly in front.")
        print()

except KeyboardInterrupt:
    print("\n\nStopped early")
finally:
    ser.close()
