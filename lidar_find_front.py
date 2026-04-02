import serial
import time

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
            if 10 < distance_cm < 300:  # 10cm to 3m
                points.append((angle, distance_cm, quality))
    
    return start_angle, points

print("="*70)
print("LIDAR REAL-TIME ANGLE TEST")
print("="*70)
print()
print("INSTRUCTIONS:")
print("1. Clear area around robot")
print("2. Place ONE object at 50cm directly IN FRONT of robot")
print("3. Watch the output - it will show angles where objects are detected")
print("4. Press Ctrl+C to stop")
print()
print("="*70)
print()

ser = serial.Serial(PORT, BAUDRATE, timeout=1)
time.sleep(1)

print("Scanning... (showing objects 10-300cm away)")
print()

try:
    update_counter = 0
    while True:
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
            
            if start_angle is not None and len(points) > 0:
                update_counter += 1
                
                # Show every 10th update to not spam too much
                if update_counter % 10 == 0:
                    # Find closest point
                    closest = min(points, key=lambda p: p[1])
                    angle, dist, qual = closest
                    
                    print(f"Closest object: {dist:6.1f}cm at angle {angle:6.1f}° (Quality: {qual})")

except KeyboardInterrupt:
    print("\n\nStopped")
    print()
    print("="*70)
    print("NEXT STEP:")
    print("  Note the angle where you saw the front object")
    print("  We'll use this to set the correct angle offset")
    print("="*70)
finally:
    ser.close()
