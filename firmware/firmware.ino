#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);

#define LEFT_SPD_PIN 16
#define LEFT_DIR1_PIN 17
#define LEFT_DIR2_PIN 18
#define RIGHT_SPD_PIN 25
#define RIGHT_DIR1_PIN 26
#define RIGHT_DIR2_PIN 27
#define ENCODER_RIGHT_A 34
#define ENCODER_RIGHT_B 35
#define ENCODER_LEFT_A 32
#define ENCODER_LEFT_B 33

#define PI_RX_PIN 23
#define PI_TX_PIN 19
#define PI_BAUD 115200
#define CMD_TIMEOUT_MS 500

volatile long right_encoder_count = 0;
volatile long left_encoder_count = 0;

const int MAX_PWM = 255;  // Same as original working firmware
const float WHEEL_RADIUS = 4.2;        
const float WHEEL_BASE = 31.5;         
const float ENCODER_TICKS_PER_REV = 207.0;  // = 207/3 (gear ratio correction)

float left_distance = 0.0;
float right_distance = 0.0;
float x = 0.0;
float y = 0.0;
float theta = 0.0;
float mpu_angle = 0.0;

long last_left_count = 0;
long last_right_count = 0;
unsigned long last_time = 0;
unsigned long sample_time = 100; // 10 Hz like original

// Commands from Python
float target_x = 0.0;
float target_y = 0.0;
bool goal_received = false;  // Don't move until goal is received
unsigned long last_cmd_time = 0;

// Go-to-goal parameters (from working firmware)
const float DISTANCE_THRESHOLD = 5.0;
float Kp_heading = 55.0;  // Same as original working firmware
float Kp_velocity = 2.0;
const int BASE_PWM = 170;  // Same as original working firmware

// Debug variables
int debug_leftPWM = 0;
int debug_rightPWM = 0;
float debug_base_speed = 0.0;
float debug_correction = 0.0;

// Encoder ISRs
void IRAM_ATTR right_encoder_ISR() {
    if (digitalRead(ENCODER_RIGHT_B)) right_encoder_count--;
    else right_encoder_count++;
}
void IRAM_ATTR left_encoder_ISR() {
    if (digitalRead(ENCODER_LEFT_B)) left_encoder_count++;
    else left_encoder_count--;
}

// Initialization
void initIMU() {
    Wire.begin();
    mpu6050.begin();
    mpu6050.calcGyroOffsets(true);
}

void initMotorPins() {
    pinMode(LEFT_SPD_PIN, OUTPUT);
    pinMode(LEFT_DIR1_PIN, OUTPUT);
    pinMode(LEFT_DIR2_PIN, OUTPUT);
    pinMode(RIGHT_SPD_PIN, OUTPUT);
    pinMode(RIGHT_DIR1_PIN, OUTPUT);
    pinMode(RIGHT_DIR2_PIN, OUTPUT);
}

void initEncoders() {
    pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);
    pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
    pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), right_encoder_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), left_encoder_ISR, RISING);
}

// Motor control with braking
void setMotor(int leftPWM, int rightPWM) {
    // LEFT
    if (leftPWM > 0) {
        digitalWrite(LEFT_DIR1_PIN, LOW);
        digitalWrite(LEFT_DIR2_PIN, HIGH);
    } else if (leftPWM < 0) {
        digitalWrite(LEFT_DIR1_PIN, HIGH);
        digitalWrite(LEFT_DIR2_PIN, LOW);
    } else {
        digitalWrite(LEFT_DIR1_PIN, HIGH);
        digitalWrite(LEFT_DIR2_PIN, HIGH); // BRAKE
    }
    // RIGHT
    if (rightPWM > 0) {
        digitalWrite(RIGHT_DIR1_PIN, LOW);
        digitalWrite(RIGHT_DIR2_PIN, HIGH);
    } else if (rightPWM < 0) {
        digitalWrite(RIGHT_DIR1_PIN, HIGH);
        digitalWrite(RIGHT_DIR2_PIN, LOW);
    } else {
        digitalWrite(RIGHT_DIR1_PIN, HIGH);
        digitalWrite(RIGHT_DIR2_PIN, HIGH); // BRAKE
    }

    analogWrite(LEFT_SPD_PIN, constrain(abs(leftPWM), 0, MAX_PWM));
    analogWrite(RIGHT_SPD_PIN, constrain(abs(rightPWM), 0, MAX_PWM));
}

// Odometry
void Odometry() {
    long delta_left  = left_encoder_count - last_left_count;
    long delta_right = right_encoder_count - last_right_count;
    last_left_count  = left_encoder_count;
    last_right_count = right_encoder_count;

    float dt = sample_time / 1000.0;
    left_distance  = (delta_left / ENCODER_TICKS_PER_REV) * (2 * PI * WHEEL_RADIUS);
    right_distance = (delta_right / ENCODER_TICKS_PER_REV) * (2 * PI * WHEEL_RADIUS);

    float delta_theta = (right_distance - left_distance) / WHEEL_BASE;
    float delta_x = ((left_distance + right_distance)/2.0) * cos(theta + delta_theta/2.0);
    float delta_y = ((left_distance + right_distance)/2.0) * sin(theta + delta_theta/2.0);

    x += delta_x;
    y += delta_y;
    theta += delta_theta;

    while (theta > PI) theta -= 2*PI;
    while (theta < -PI) theta += 2*PI;
}

// Read UART from Pi
// Read goal from Python: "tx:<x>,ty:<y>\n"
void readPiCommand() {
    static char buf[32];
    static uint8_t idx = 0;
    while (Serial2.available()) {
        char c = Serial2.read();
        if (c == '\n') {
            buf[idx] = '\0';
            char *tx_ptr = strstr(buf, "tx:");
            char *ty_ptr = strstr(buf, ",ty:");
            if (tx_ptr && ty_ptr) {
                target_x = atof(tx_ptr+3);
                target_y = atof(ty_ptr+4);
                goal_received = true;  // Enable movement
                last_cmd_time = millis();
            }
            idx = 0;
        } else if (idx < sizeof(buf)-1) buf[idx++] = c;
    }
}

// Send state to Pi: x,y,theta,mpu_angle,leftPWM,rightPWM,base_speed,correction
void sendState() {
    Serial2.print(x,3); Serial2.print(',');
    Serial2.print(y,3); Serial2.print(',');
    Serial2.print(theta,4); Serial2.print(',');
    Serial2.print(mpu_angle,4); Serial2.print(',');
    Serial2.print(debug_leftPWM); Serial2.print(',');
    Serial2.print(debug_rightPWM); Serial2.print(',');
    Serial2.print(debug_base_speed,1); Serial2.print(',');
    Serial2.print(debug_correction,1);
    Serial2.println();
}

// PID velocity control
// Direct PWM go-to-goal (from working firmware)
void goToGoal() {
    // Don't move until we receive a goal from Python
    if (!goal_received) {
        setMotor(0, 0);
        return;
    }
    
    float distance_to_goal = sqrt(pow(target_x - x, 2) + pow(target_y - y, 2));
    
    if (distance_to_goal > DISTANCE_THRESHOLD) {
        float base_speed = Kp_velocity * distance_to_goal;
        base_speed = constrain(base_speed, 0, BASE_PWM);
        
        float desired_heading = atan2(target_y - y, target_x - x);
        float heading_error = desired_heading - mpu_angle;
        while (heading_error > PI) heading_error -= 2 * PI;
        while (heading_error < -PI) heading_error += 2 * PI;
        
        float correction = Kp_heading * heading_error;

        int leftPWM = base_speed - correction;
        int rightPWM = base_speed + correction;
        
        // Enforce minimum PWM to overcome friction
        if (leftPWM > 0 && leftPWM < 80) leftPWM = 80;
        else if (leftPWM < 0 && leftPWM > -80) leftPWM = -80;
        
        if (rightPWM > 0 && rightPWM < 80) rightPWM = 80;
        else if (rightPWM < 0 && rightPWM > -80) rightPWM = -80;
        
        leftPWM = constrain(leftPWM, -MAX_PWM, MAX_PWM);
        rightPWM = constrain(rightPWM, -MAX_PWM, MAX_PWM);
        
        // Store for debug output to Pi
        debug_leftPWM = leftPWM;
        debug_rightPWM = rightPWM;
        debug_base_speed = base_speed;
        debug_correction = correction;
        
        setMotor(leftPWM, rightPWM);
    } else {
        setMotor(0, 0);
    }
}

// Arduino setup
void setup() {
    Serial.begin(115200);
    Serial2.begin(PI_BAUD, SERIAL_8N1, PI_RX_PIN, PI_TX_PIN);
    initIMU();
    initMotorPins();
    initEncoders();
    Serial.println("System initialized.");
}

// Arduino loop
void loop() {
    mpu6050.update();
    mpu_angle = mpu6050.getAngleZ() * PI / 180.0;
    while (mpu_angle > PI) mpu_angle -= 2*PI;
    while (mpu_angle < -PI) mpu_angle += 2*PI;

    readPiCommand();  // Read new goals from Python
    
    unsigned long current_time = millis();
    if (current_time - last_time >= sample_time) {
        Odometry();
        goToGoal();  // Direct PWM control
        sendState();  // Send state to Python
        
        // Debug to USB Serial
        Serial.print(x, 2); Serial.print(" ");
        Serial.print(y, 2); Serial.print(" ");
        Serial.print(theta, 4); Serial.print(" ");
        Serial.print(mpu_angle, 4); Serial.println();
        
        last_time = current_time;
    }
}