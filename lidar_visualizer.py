import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import math

# ----------------------------
# Configuration
# ----------------------------
PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
PACKET_HEADER = 0xAA
DISTANCE_SCALE = 0.001  # mm to meters
POINTS_PER_PACKET = 12
MAX_POINTS = 360  # Keep one full rotation
MAX_DISTANCE = 6.0  # meters

# Store recent points
point_buffer = deque(maxlen=MAX_POINTS)

# ----------------------------
# Parse LiDAR packet
# ----------------------------
def parse_packet(packet_bytes):
    points = []
    for i in range(2, len(packet_bytes)-2, 3):
        dist_raw = packet_bytes[i] | (packet_bytes[i+1] << 8)
        quality = packet_bytes[i+2]
        distance = dist_raw * DISTANCE_SCALE
        if 0.02 < distance < MAX_DISTANCE:
            points.append((distance, quality))
        else:
            points.append((0, 0))
    return points

# ----------------------------
# Read LiDAR data
# ----------------------------
def read_lidar_data(ser):
    angle_step = 360 / POINTS_PER_PACKET
    angle_offset = 0
    
    while True:
        byte = ser.read(1)
        if not byte:
            continue
        if byte[0] == PACKET_HEADER:
            packet = ser.read(11)
            if len(packet) != 11:
                continue
            full_packet = bytearray([PACKET_HEADER]) + packet
            points = parse_packet(full_packet)
            
            for i, (distance, quality) in enumerate(points):
                if distance > 0:
                    angle = (angle_offset + i * angle_step) % 360
                    point_buffer.append((angle, distance))
            
            angle_offset = (angle_offset + len(points) * angle_step) % 360
            break

# ----------------------------
# Matplotlib setup
# ----------------------------
fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection='polar')
ax.set_ylim(0, MAX_DISTANCE)
ax.set_theta_zero_location('N')  # 0° at top
ax.set_theta_direction(-1)  # Clockwise
ax.grid(True)
ax.set_title('YDLidar X3 Real-Time Visualization', pad=20)

scatter = ax.scatter([], [], c='blue', s=2, alpha=0.6)

# ----------------------------
# Animation update function
# ----------------------------
def update(frame):
    read_lidar_data(ser)
    
    if len(point_buffer) > 0:
        angles = [math.radians(angle) for angle, _ in point_buffer]
        distances = [distance for _, distance in point_buffer]
        
        scatter.set_offsets(np.c_[angles, distances])
        
        # Color by distance (optional)
        colors = plt.cm.viridis(np.array(distances) / MAX_DISTANCE)
        scatter.set_color(colors)
    
    return scatter,

# ----------------------------
# Main
# ----------------------------
if __name__ == "__main__":
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        print(f"Connected to LiDAR on {PORT}")
        print("Visualizing... Close window to stop.")
        
        ani = FuncAnimation(fig, update, interval=50, blit=True, cache_frame_data=False)
        plt.show()
        
    except KeyboardInterrupt:
        print("\nStopping...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'ser' in locals():
            ser.close()
        print("LiDAR disconnected")
