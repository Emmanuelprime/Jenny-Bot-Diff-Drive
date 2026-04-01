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
MAX_POINTS = 1500  # Keep multiple full rotations for smooth 360° view
MAX_DISTANCE = 6.0  # meters

# Store recent points
point_buffer = deque(maxlen=MAX_POINTS)

# ----------------------------
# Parse LiDAR packet
# ----------------------------
def parse_packet(packet_bytes):
    # X3 packet structure:
    # Byte 0: Header (0xAA)
    # Byte 1: Start angle LSB (in 0.01 degree units)
    # Byte 2-37: Distance/Quality pairs (12 points, 3 bytes each)
    
    start_angle_raw = packet_bytes[1]
    start_angle = start_angle_raw * 0.01  # Convert to degrees
    
    points = []
    for i in range(2, len(packet_bytes)-2, 3):
        dist_raw = packet_bytes[i] | (packet_bytes[i+1] << 8)
        quality = packet_bytes[i+2]
        distance = dist_raw * DISTANCE_SCALE
        if 0.02 < distance < MAX_DISTANCE:
            points.append((distance, quality))
        else:
            points.append((0, 0))
    return start_angle, points

# ----------------------------
# Read LiDAR data
# ----------------------------
def read_lidar_data(ser):
    angle_step = 360.0 / POINTS_PER_PACKET
    
    # Read multiple packets to fill buffer faster
    packets_read = 0
    
    while packets_read < 5:  # Read 5 packets per update (60 points)
        byte = ser.read(1)
        if not byte:
            continue
        if byte[0] == PACKET_HEADER:
            packet = ser.read(11)
            if len(packet) != 11:
                continue
            full_packet = bytearray([PACKET_HEADER]) + packet
            start_angle, points = parse_packet(full_packet)
            
            for i, (distance, quality) in enumerate(points):
                if distance > 0:
                    # Use actual start angle from packet + offset for each point
                    angle = (start_angle + i * angle_step) % 360
                    point_buffer.append((angle, distance))
            
            packets_read += 1

# ----------------------------
# Matplotlib setup (XY plot)
# ----------------------------
fig, ax = plt.subplots(figsize=(10, 10))
ax.set_xlim(-MAX_DISTANCE, MAX_DISTANCE)
ax.set_ylim(-MAX_DISTANCE, MAX_DISTANCE)
ax.set_aspect('equal')
ax.grid(True, alpha=0.3)
ax.set_xlabel('X (meters)')
ax.set_ylabel('Y (meters)')
ax.set_title('YDLidar X3 - XY View (Robot at Origin)')

# Draw robot at center
robot_circle = plt.Circle((0, 0), 0.15, color='red', fill=True, alpha=0.5, label='Robot')
ax.add_patch(robot_circle)
# Draw robot direction (forward = +Y)
ax.arrow(0, 0, 0, 0.3, head_width=0.1, head_length=0.1, fc='red', ec='red')

scatter = ax.scatter([], [], c='blue', s=10, alpha=0.6)
ax.legend()

# ----------------------------
# Animation update function
# ----------------------------
def update(frame):
    read_lidar_data(ser)
    
    if len(point_buffer) > 0:
        x_coords = []
        y_coords = []
        
        for angle_deg, distance in point_buffer:
            # Convert polar to Cartesian
            # 0° = North (+Y axis), rotating clockwise
            angle_rad = math.radians(angle_deg)
            x = distance * math.sin(angle_rad)
            y = distance * math.cos(angle_rad)
            x_coords.append(x)
            y_coords.append(y)
        
        scatter.set_offsets(np.c_[x_coords, y_coords])
        
        # Color by distance
        distances = [distance for _, distance in point_buffer]
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
        print("Visualizing XY view... Close window to stop.")
        
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
