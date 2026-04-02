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
            
            if 2 < distance_cm < 300:
                points.append((angle, distance_cm, quality))
    
    return start_angle, points

print(f"Connecting to LiDAR on {PORT}...")
ser = serial.Serial(PORT, BAUDRATE, timeout=1)
time.sleep(1)

print("\nCollecting LiDAR data for 3 seconds...")
print("Showing angle distribution to help identify LiDAR orientation\n")

angle_bins = defaultdict(list)
packets_read = 0
start_time = time.time()

try:
    while time.time() - start_time < 3.0:
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
                    # Bin angles into 30° sectors
                    sector = int(angle / 30) * 30
                    angle_bins[sector].append(distance)
                
                packets_read += 1

    print(f"Processed {packets_read} packets\n")
    print("="*70)
    print("ANGLE DISTRIBUTION (shows where LiDAR sees objects)")
    print("="*70)
    print()
    
    # Sort and display by sector
    for sector in sorted(angle_bins.keys()):
        points_in_sector = angle_bins[sector]
        count = len(points_in_sector)
        avg_dist = sum(points_in_sector) / count if count > 0 else 0
        min_dist = min(points_in_sector) if count > 0 else 0
        
        # Visual bar
        bar_length = min(50, count // 5)
        bar = '#' * bar_length
        
        print(f"{sector:3d}°-{sector+29:3d}°: {count:4d} points | Avg:{avg_dist:6.1f}cm Min:{min_dist:6.1f}cm | {bar}")
    
    print()
    print("="*70)
    print("HOW TO INTERPRET:")
    print("  - Place object directly in FRONT of robot")
    print("  - Look for sector with lowest Min distance")
    print("  - That sector angle = LiDAR's \"front\" in degrees")
    print()
    print("EXAMPLE:")
    print("  If you see min distance at 90°-119°, then LiDAR front = ~90°")
    print("  Update LIDAR_ANGLE_OFFSET in your detection scripts")
    print("="*70)
    
except KeyboardInterrupt:
    print("\nStopped")
finally:
    ser.close()
