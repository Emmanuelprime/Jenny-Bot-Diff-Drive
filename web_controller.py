from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
import serial
import math
import time
import threading
import json

app = Flask(__name__)
app.config['SECRET_KEY'] = 'jenny-bot-secret'
socketio = SocketIO(app, cors_allowed_origins="*")

SERIAL_PORT = '/dev/serial0'
BAUD_RATE = 115200

WHEEL_BASE = 31.5
MAX_LINEAR_VEL = 50.0
MAX_ANGULAR_VEL = 2.0

Kp_linear = 2.51
Kp_angular = 8.5
DISTANCE_THRESHOLD = 5.0
HEADING_THRESHOLD = 0.087

LOOP_HZ = 10
LOOP_DT = 1.0 / LOOP_HZ

class RobotController:
    def __init__(self):
        self.ser = None
        self.running = False
        self.waypoints = []
        self.waypoint_index = 0
        self.target_x = 0.0
        self.target_y = 0.0
        self.desired_theta = None
        self.position_reached = False
        self.orientation_reached = False
        self.thread = None
        
    def connect(self):
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)
            return True
        except Exception as e:
            print(f"Serial connection error: {e}")
            return False
    
    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.write(b'v:0.0,w:0.0\n')
            time.sleep(0.1)
            self.ser.close()
    
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def parse_state(self, line):
        try:
            parts = line.strip().split(',')
            if len(parts) != 6:
                return None
            return tuple(float(p) for p in parts)
        except:
            return None
    
    def go_to_goal(self, x, y, fused_theta):
        dx = self.target_x - x
        dy = self.target_y - y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < DISTANCE_THRESHOLD:
            return 0.0, 0.0, True
        
        v = Kp_linear * distance
        v = max(0.0, min(v, MAX_LINEAR_VEL))
        
        desired_heading = math.atan2(dy, dx)
        heading_error = self.normalize_angle(desired_heading - fused_theta)
        omega = Kp_angular * heading_error
        omega = max(-MAX_ANGULAR_VEL, min(omega, MAX_ANGULAR_VEL))
        return v, omega, False
    
    def align_orientation(self, fused_theta):
        heading_error = self.normalize_angle(self.desired_theta - fused_theta)
        
        if abs(heading_error) < HEADING_THRESHOLD:
            return 0.0, True
        
        omega = Kp_angular * heading_error
        omega = max(-MAX_ANGULAR_VEL, min(omega, MAX_ANGULAR_VEL))
        return omega, False
    
    def start_navigation(self, waypoints):
        if self.running:
            return False
        
        self.waypoints = waypoints
        self.waypoint_index = 0
        self.target_x, self.target_y, self.desired_theta = waypoints[0]
        self.position_reached = False
        self.orientation_reached = False
        self.running = True
        
        self.thread = threading.Thread(target=self.navigation_loop)
        self.thread.daemon = True
        self.thread.start()
        return True
    
    def stop_navigation(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.write(b'v:0.0,w:0.0\n')
    
    def navigation_loop(self):
        all_waypoints_reached = False
        
        while self.running and not all_waypoints_reached:
            loop_start = time.time()
            
            if not self.ser or not self.ser.is_open:
                break
            
            raw = self.ser.readline().decode('utf-8', errors='ignore')
            state = self.parse_state(raw) if raw else None
            
            if state is None:
                cmd = b'v:0.0,w:0.0\n'
                self.ser.write(cmd)
                time.sleep(max(0.0, LOOP_DT - (time.time() - loop_start)))
                continue
            
            x, y, fused_theta, mpu_angle, lv, rv = state
            
            dx = self.target_x - x
            dy = self.target_y - y
            dist = math.sqrt(dx*dx + dy*dy)
            
            if not self.position_reached:
                desired_heading = math.atan2(dy, dx)
                heading_error = self.normalize_angle(desired_heading - fused_theta)
                v, omega, self.position_reached = self.go_to_goal(x, y, fused_theta)
                
                if self.position_reached and self.desired_theta is None:
                    self.orientation_reached = True
            
            elif not self.orientation_reached and self.desired_theta is not None:
                heading_error = self.normalize_angle(self.desired_theta - fused_theta)
                omega, self.orientation_reached = self.align_orientation(fused_theta)
                v = 0.0
            else:
                v, omega = 0.0, 0.0
                heading_error = 0.0
                
                if self.waypoint_index < len(self.waypoints) - 1:
                    self.waypoint_index += 1
                    wp = self.waypoints[self.waypoint_index]
                    self.target_x, self.target_y = wp[0], wp[1]
                    self.desired_theta = wp[2]
                    self.position_reached = False
                    self.orientation_reached = False
                else:
                    all_waypoints_reached = True
            
            status = "ALIGN" if self.position_reached and not self.orientation_reached else "MOVE"
            
            robot_state = {
                'x': round(x, 2),
                'y': round(y, 2),
                'theta': round(math.degrees(fused_theta), 2),
                'target_x': round(self.target_x, 2),
                'target_y': round(self.target_y, 2),
                'distance': round(dist, 2),
                'heading_error': round(math.degrees(heading_error), 2),
                'v': round(v, 2),
                'omega': round(omega, 2),
                'status': status,
                'waypoint': self.waypoint_index + 1,
                'total_waypoints': len(self.waypoints),
                'completed': all_waypoints_reached
            }
            
            socketio.emit('robot_state', robot_state)
            
            cmd = f"v:{v:.3f},w:{omega:.4f}\n"
            self.ser.write(cmd.encode())
            
            elapsed = time.time() - loop_start
            time.sleep(max(0.0, LOOP_DT - elapsed))
        
        self.running = False
        socketio.emit('navigation_complete', {'message': 'All waypoints reached'})

controller = RobotController()

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/connect', methods=['POST'])
def connect():
    if controller.connect():
        return jsonify({'status': 'success', 'message': 'Connected to robot'})
    return jsonify({'status': 'error', 'message': 'Failed to connect'}), 500

@app.route('/api/disconnect', methods=['POST'])
def disconnect():
    controller.disconnect()
    return jsonify({'status': 'success', 'message': 'Disconnected'})

@app.route('/api/start', methods=['POST'])
def start_navigation():
    data = request.json
    waypoints = data.get('waypoints', [])
    
    if not waypoints:
        return jsonify({'status': 'error', 'message': 'No waypoints provided'}), 400
    
    parsed_waypoints = []
    for wp in waypoints:
        x = float(wp['x'])
        y = float(wp['y'])
        theta = math.radians(float(wp['theta'])) if wp.get('theta') is not None else None
        parsed_waypoints.append((x, y, theta))
    
    if controller.start_navigation(parsed_waypoints):
        return jsonify({'status': 'success', 'message': 'Navigation started'})
    return jsonify({'status': 'error', 'message': 'Navigation already running'}), 400

@app.route('/api/stop', methods=['POST'])
def stop_navigation():
    controller.stop_navigation()
    return jsonify({'status': 'success', 'message': 'Navigation stopped'})

@socketio.on('connect')
def handle_connect():
    emit('connected', {'message': 'Connected to server'})

@socketio.on('disconnect')
def handle_disconnect():
    print('Client disconnected')

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=5000, debug=True)
