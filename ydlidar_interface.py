import serial
import time

# ----------------------------
# Configuration
# ----------------------------
PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
PACKET_HEADER = 0xAA  # X3 start byte
DISTANCE_SCALE = 0.001  # mm to meters
POINTS_PER_PACKET = 12  # X3 sends 12 measurements per packet

# ----------------------------
# Helper function to parse a single packet
# ----------------------------
def parse_packet(packet_bytes):
    points = []
    for i in range(2, len(packet_bytes)-2, 3):
        dist_raw = packet_bytes[i] | (packet_bytes[i+1] << 8)
        quality = packet_bytes[i+2]
        distance = dist_raw * DISTANCE_SCALE
        if 0.02 < distance < 6.0:
            points.append((distance, quality))
        else:
            points.append((0, 0))
    return points

# ----------------------------
# Generator for streaming LiDAR points
# ----------------------------
def stream_lidar(serial_port, points_per_packet=POINTS_PER_PACKET):
    angle_step = 360 / points_per_packet
    angle_offset = 0
    while True:
        byte = serial_port.read(1)
        if not byte:
            continue
        if byte[0] == PACKET_HEADER:
            packet = serial_port.read(11)
            if len(packet) != 11:
                continue
            full_packet = bytearray([PACKET_HEADER]) + packet
            points = parse_packet(full_packet)
            for i, (distance, quality) in enumerate(points):
                angle = (angle_offset + i * angle_step) % 360
                yield angle, distance, quality
            angle_offset = (angle_offset + len(points) * angle_step) % 360

# ----------------------------
# Usage example
# ----------------------------
if __name__ == "__main__":
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    print("Streaming LiDAR points... Press Ctrl+C to stop.")
    try:
        for angle, distance, quality in stream_lidar(ser):
            # Process each point immediately
            print(f"Angle: {angle:.1f}°, Distance: {distance:.3f} m, Quality: {quality}")
            # Optional: add a small sleep if your processing is too fast
            # time.sleep(0.001)
    except KeyboardInterrupt:
        print("Stopping LiDAR...")
        ser.close()