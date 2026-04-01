import serial
import time

# ----------------------------
# Configuration
# ----------------------------
PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
PACKET_HEADER = 0xAA
DISTANCE_SCALE = 0.001  # mm to meters
POINTS_PER_PACKET = 12

# ----------------------------
# Parse LiDAR packet
# ----------------------------
def parse_packet(packet_bytes):
    start_angle_raw = packet_bytes[1] | (packet_bytes[2] << 8)
    start_angle = (start_angle_raw >> 1) / 64.0
    
    points = []
    for i in range(3, len(packet_bytes)-1, 3):
        if i + 2 < len(packet_bytes):
            dist_raw = packet_bytes[i] | (packet_bytes[i+1] << 8)
            quality = packet_bytes[i+2]
            distance = dist_raw * DISTANCE_SCALE
            if 0.02 < distance < 6.0:
                points.append((distance, quality))
            else:
                points.append((0, 0))
    return start_angle, points

# ----------------------------
# Collect one full 360° scan
# ----------------------------
def collect_full_scan(ser):
    scan_data = {}  # angle -> (distance, quality)
    angle_step = 360.0 / POINTS_PER_PACKET
    
    start_collection = None
    packets_collected = 0
    max_packets = 50  # Should cover multiple full rotations
    
    print("Collecting 360° scan data...")
    print("Please wait...\n")
    
    while packets_collected < max_packets:
        byte = ser.read(1)
        if not byte:
            continue
        if byte[0] == PACKET_HEADER:
            packet = ser.read(11)
            if len(packet) != 11:
                continue
            
            full_packet = bytearray([PACKET_HEADER]) + packet
            start_angle, points = parse_packet(full_packet)
            
            if start_collection is None:
                start_collection = start_angle
            
            for i, (distance, quality) in enumerate(points):
                angle = (start_angle + i * angle_step) % 360
                angle_key = int(angle)  # Round to nearest degree
                
                # Store the measurement (keep most recent)
                if distance > 0:
                    scan_data[angle_key] = (distance, quality)
            
            packets_collected += 1
            
            # Check if we've covered full 360° range
            if len(scan_data) > 300:  # Should have most angles covered
                break
    
    return scan_data

# ----------------------------
# Display scan data
# ----------------------------
def display_scan(scan_data):
    print("=" * 70)
    print("COMPLETE 360° LIDAR SCAN")
    print("=" * 70)
    print()
    
    # Display by 30° sectors to identify front
    print("SECTOR ANALYSIS (to find front of LiDAR):")
    print("-" * 70)
    
    sectors = {}
    for angle in range(0, 360, 30):
        sector_points = []
        for a in range(angle, angle + 30):
            if a in scan_data:
                dist, qual = scan_data[a]
                sector_points.append(dist)
        
        if sector_points:
            avg_dist = sum(sector_points) / len(sector_points)
            min_dist = min(sector_points)
            count = len(sector_points)
            sectors[angle] = (avg_dist, min_dist, count)
            print(f"{angle:3d}°-{angle+29:3d}°: Avg={avg_dist:.2f}m, Min={min_dist:.2f}m, Points={count}")
        else:
            print(f"{angle:3d}°-{angle+29:3d}°: No data")
    
    print()
    print("=" * 70)
    print("DETAILED ANGLE DATA:")
    print("=" * 70)
    print()
    
    # Display all measurements sorted by angle
    sorted_angles = sorted(scan_data.keys())
    
    for i in range(0, len(sorted_angles), 10):
        line_angles = sorted_angles[i:i+10]
        print("Angles:    " + "  ".join(f"{a:3d}°" for a in line_angles))
        print("Distances: " + "  ".join(f"{scan_data[a][0]:.2f}" for a in line_angles))
        print()
    
    print("=" * 70)
    print(f"Total unique angles captured: {len(scan_data)}")
    print()
    print("INSTRUCTIONS TO FIND FRONT:")
    print("1. Place an object (book/box) 50cm in front of LiDAR")
    print("2. Look for the sector with minimum distance around 0.5m")
    print("3. That sector angle range = front of the LiDAR")
    print("=" * 70)

# ----------------------------
# Main
# ----------------------------
if __name__ == "__main__":
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        print(f"Connected to LiDAR on {PORT}\n")
        
        # Wait for LiDAR to stabilize
        time.sleep(1)
        
        scan_data = collect_full_scan(ser)
        display_scan(scan_data)
        
    except KeyboardInterrupt:
        print("\nStopped by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'ser' in locals():
            ser.close()
        print("\nLiDAR disconnected")
