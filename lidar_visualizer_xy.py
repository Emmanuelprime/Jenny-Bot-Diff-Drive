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
# Parse LiDAR packet (X3 protocol)
# ----------------------------
def parse_packet(packet_bytes):
    # X3 packet: 0xAA 0x55 CT LSN FSA(2) LSA(2) CS(2) [Data(3*LSN)]
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
            distance = dist_raw * 0.25 * DISTANCE_SCALE
            angle = (start_angle + i * angle_step) % 360
            
            if 0.02 < distance < MAX_DISTANCE:
                points.append((angle, distance, quality))
    
    return start_angle, points

# ----------------------------
# Read LiDAR data
# ----------------------------
def read_lidar_data(ser):
    packets_read = 0
    
    while packets_read < 3:  # Read 3 packets per update
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
