import serial
import math
import time
import sys

SERIAL_PORT = '/dev/serial0'
BAUD_RATE   = 115200

LOOP_HZ  = 10
LOOP_DT  = 1.0 / LOOP_HZ

def parse_state(line):
    """Parse ESP32 state: x,y,theta,mpu_angle"""
    try:
        parts = line.strip().split(',')
        if len(parts) != 4:
            return None
        return tuple(float(p) for p in parts)
    except:
        return None

def main(target_x, target_y):
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)
    print(f"Connected to ESP32 on {SERIAL_PORT}")
    print(f"Target: ({target_x}, {target_y}) cm")
    print("ESP32 handles all control - Python just monitors\n")
    
    # Send goal once
    cmd = f"tx:{target_x},ty:{target_y}\n"
    ser.write(cmd.encode())
    print(f"Sent goal: {cmd.strip()}")
    
    goal_reached = False

    while True:
        loop_start = time.time()
        
        # Read state from ESP32
        raw = ser.readline().decode('utf-8', errors='ignore')
        state = parse_state(raw) if raw else None

        if state is None:
            time.sleep(max(0.0, LOOP_DT - (time.time() - loop_start)))
            continue

        x, y, theta, mpu_angle = state
        
        # Calculate distance to goal
        dx = target_x - x
        dy = target_y - y
        dist = math.sqrt(dx * dx + dy * dy)
        
        print(f"Pos:({x:.1f},{y:.1f}) θ:{math.degrees(mpu_angle):.1f}° Dist:{dist:.1f}cm", end='')
        
        if dist < 5.0 and not goal_reached:
            print(" ← GOAL REACHED!", end='')
            goal_reached = True
        
        print()  # New line
        
        # Hold loop rate
        elapsed = time.time() - loop_start
        time.sleep(max(0.0, LOOP_DT - elapsed))

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python controller_simple.py <target_x> <target_y>")
        print("Example: python controller_simple.py 100 0")
        sys.exit(1)
    
    try:
        target_x = float(sys.argv[1])
        target_y = float(sys.argv[2])
    except ValueError:
        print("Error: target_x and target_y must be numbers")
        sys.exit(1)
    
    try:
        main(target_x, target_y)
    except KeyboardInterrupt:
        print("\nStopped.")
