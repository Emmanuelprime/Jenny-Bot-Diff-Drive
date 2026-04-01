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
# Parse LiDAR packet (X3 protocol)
# ----------------------------
def parse_packet(packet_bytes):
    # X3 packet: 0xAA 0x55 CT LSN FSA(2) LSA(2) CS(2) [Data(3*LSN)]
    # CT = packet type
    # LSN = number of sample points (usually 8)
    # FSA = start angle (16-bit)
    # LSA = end angle (16-bit)
    # CS = checksum (2 bytes)
    # Data = distance(2) + intensity(1) for each sample
    
    if len(packet_bytes) < 10:
        return None, []
    
    ct = packet_bytes[2]
    lsn = packet_bytes[3]  # Number of samples
    
    fsa_raw = packet_bytes[4] | (packet_bytes[5] << 8)
    lsa_raw = packet_bytes[6] | (packet_bytes[7] << 8)
    
    # Decode angles (X3 format: angle = raw / 64)
    start_angle = (fsa_raw >> 1) / 64.0
    end_angle = (lsa_raw >> 1) / 64.0
    
    # Calculate angle step between samples
    angle_diff = end_angle - start_angle
    if angle_diff < 0:
        angle_diff += 360
    
    if lsn > 1:
        angle_step = angle_diff / (lsn - 1)
    else:
        angle_step = 0
    
    # Parse distance data (starts at byte 10)
    points = []
    data_start = 10
    
    for i in range(lsn):
        idx = data_start + i * 3
        if idx + 2 < len(packet_bytes):
            dist_raw = packet_bytes[idx] | (packet_bytes[idx+1] << 8)
            quality = packet_bytes[idx+2]
            distance = dist_raw * 0.25 * DISTANCE_SCALE  # X3 uses 0.25mm units
            
            angle = (start_angle + i * angle_step) % 360
            
            if 0.02 < distance < 6.0:
                points.append((angle, distance, quality))
            else:
                points.append((angle, 0, 0))
    
    return start_angle, points

# ----------------------------
# Collect one full 360° scan
# ----------------------------
def collect_full_scan(ser):
    scan_data = {}  # angle -> (distance, quality)
    
    start_collection = None
    packets_collected = 0
    max_packets = 100  # Should cover multiple full rotations
    
    print("Collecting 360° scan data...")
    print("Please wait...\n")
    
    while packets_collected < max_packets:
        byte = ser.read(1)
        if not byte:
            continue
        
        # Look for packet header 0xAA 0x55
        if byte[0] == 0xAA:
            second_byte = ser.read(1)
            if not second_byte or second_byte[0] != 0x55:
                continue
            
            # Read rest of header: CT, LSN, FSA(2), LSA(2), CS(2) = 8 bytes
            header = ser.read(8)
            if len(header) != 8:
                continue
            
            lsn = header[1]  # Number of sample points
            data_bytes = lsn * 3  # Each sample is 3 bytes
            
            # Read the data section
            data = ser.read(data_bytes)
            if len(data) != data_bytes:
                continue
            
            # Construct full packet
            full_packet = bytearray([0xAA, 0x55]) + header + data
            
            start_angle, points = parse_packet(full_packet)
            
            if start_angle is None:
                continue
            
            for angle, distance, quality in points:
                angle_key = int(round(angle))  # Round to nearest degree
                
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
