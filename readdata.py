import serial
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

max_points = 500
x_data = deque(maxlen=max_points)
y_data = deque(maxlen=max_points)
theta_data = deque(maxlen=max_points)
mpu_data = deque(maxlen=max_points)
time_data = deque(maxlen=max_points)

ser = None
start_time = time.time()

def init_serial(port='COM3', baudrate=115200):
    global ser
    try:
        ser = serial.Serial(port, baudrate, timeout=0.1)
        print(f"Connected to {port} at {baudrate} baud.")
        return True
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        return False

def read_data():
    global ser, start_time
    if ser and ser.is_open:
        try:
            line = ser.readline().decode('utf-8').strip()
            if line:
                parts = line.split()
                if len(parts) == 4:
                    x = float(parts[0])
                    y = float(parts[1])
                    theta = float(parts[2])
                    mpu_angle = float(parts[3])
                    
                    current_time = time.time() - start_time
                    x_data.append(x)
                    y_data.append(y)
                    theta_data.append(theta)
                    mpu_data.append(mpu_angle)
                    time_data.append(current_time)
                    
                    return True
        except (ValueError, IndexError) as e:
            pass
    return False

def plot_realtime(port='COM3', baudrate=115200):
    if not init_serial(port, baudrate):
        return
    
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12, 10))

    ax1.set_xlabel('X (cm)')
    ax1.set_ylabel('Y (cm)')
    ax1.set_title('Robot Trajectory')
    ax1.grid(True)
    ax1.axis('equal')
    
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Angle (rad)')
    ax2.set_title('Odometry Theta vs Time')
    ax2.grid(True)
    
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Angle (rad)')
    ax3.set_title('MPU Angle vs Time')
    ax3.grid(True)
    
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Angle Error (rad)')
    ax4.set_title('Theta - MPU Angle')
    ax4.grid(True)
    
    line1, = ax1.plot([], [], 'b-', linewidth=2, label='Path')
    point1, = ax1.plot([], [], 'ro', markersize=8, label='Current')
    
    line2, = ax2.plot([], [], 'g-', linewidth=2)
    line3, = ax3.plot([], [], 'r-', linewidth=2)
    line4, = ax4.plot([], [], 'm-', linewidth=2)
    
    ax1.legend()
    
    def update(frame):
        read_data()
        
        if len(x_data) > 0:
            line1.set_data(x_data, y_data)
            point1.set_data([x_data[-1]], [y_data[-1]])
            ax1.relim()
            ax1.autoscale_view()
            
            line2.set_data(time_data, theta_data)
            ax2.relim()
            ax2.autoscale_view()
            
            line3.set_data(time_data, mpu_data)
            ax3.relim()
            ax3.autoscale_view()
            
            if len(theta_data) > 0:
                error = [t - m for t, m in zip(theta_data, mpu_data)]
                line4.set_data(time_data, error)
                ax4.relim()
                ax4.autoscale_view()
        
        return line1, point1, line2, line3, line4
    
    ani = animation.FuncAnimation(fig, update, interval=50, blit=True, cache_frame_data=False)
    
    plt.tight_layout()
    
    try:
        plt.show()
    except KeyboardInterrupt:
        print("Stopping plot...")
    finally:
        if ser and ser.is_open:
            ser.close()
            print("Serial connection closed.")

if __name__ == "__main__":
    plot_realtime()
