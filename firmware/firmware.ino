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

// UART to Raspberry Pi Zero
#define PI_RX_PIN 23
#define PI_TX_PIN 19
#define PI_BAUD 115200

// Safety watchdog: stop motors if no command received within this time
#define CMD_TIMEOUT_MS 500

volatile long right_encoder_count = 0;
volatile long left_encoder_count = 0;

const int MAX_PWM = 255;

const float WHEEL_RADIUS = 4.2;        // cm
const float WHEEL_BASE = 31.5;         // cm
const float ENCODER_TICKS_PER_REV = 207.0;
const float MAX_LINEAR_VEL = 50.0;     // cm/s

float left_distance = 0.0;
float right_distance = 0.0;
float x = 0.0;
float y = 0.0;
float theta = 0.0;
float mpu_angle = 0.0;

long last_left_count = 0;
long last_right_count = 0;
unsigned long last_time = 0;
unsigned long sample_time = 100; // ms

// Commands received from Pi
float cmd_v = 0.0;   // linear velocity  cm/s
float cmd_w = 0.0;   // angular velocity rad/s
unsigned long last_cmd_time = 0;

// Wheel velocities
float left_velocity = 0.0;        // measured cm/s
float right_velocity = 0.0;       // measured cm/s
float target_left_velocity = 0.0; // desired  cm/s
float target_right_velocity = 0.0;

// PID - left wheel
float left_error_sum = 0.0;
float left_last_error = 0.0;
float Kp_left = 5.0;
float Ki_left = 0.05;
float Kd_left = 0.1;

// PID - right wheel
float right_error_sum = 0.0;
float right_last_error = 0.0;
float Kp_right = 5.0;
float Ki_right = 0.05;
float Kd_right = 0.1;

void IRAM_ATTR right_encoder_ISR() {
    if (digitalRead(ENCODER_RIGHT_B)) right_encoder_count--;
    else right_encoder_count++;
}

void IRAM_ATTR left_encoder_ISR() {
    if (digitalRead(ENCODER_LEFT_B)) left_encoder_count++;
    else left_encoder_count--;
}

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

void setMotor(int leftPWM, int rightPWM) {
    if (leftPWM > 0) {
        digitalWrite(LEFT_DIR1_PIN, LOW);
        digitalWrite(LEFT_DIR2_PIN, HIGH);
    } else if (leftPWM < 0) {
        digitalWrite(LEFT_DIR1_PIN, HIGH);
        digitalWrite(LEFT_DIR2_PIN, LOW);
    } else {
        digitalWrite(LEFT_DIR1_PIN, LOW);
        digitalWrite(LEFT_DIR2_PIN, LOW);
    }
    
    if (rightPWM > 0) {
        digitalWrite(RIGHT_DIR1_PIN, LOW);
        digitalWrite(RIGHT_DIR2_PIN, HIGH);
    } else if (rightPWM < 0) {
        digitalWrite(RIGHT_DIR1_PIN, HIGH);
        digitalWrite(RIGHT_DIR2_PIN, LOW);
    } else {
        digitalWrite(RIGHT_DIR1_PIN, LOW);
        digitalWrite(RIGHT_DIR2_PIN, LOW);
    }
    
    analogWrite(LEFT_SPD_PIN, constrain(abs(leftPWM), 0, MAX_PWM));
    analogWrite(RIGHT_SPD_PIN, constrain(abs(rightPWM), 0, MAX_PWM));
}

void setup() {
    Serial.begin(115200);   // debug
    Serial2.begin(PI_BAUD, SERIAL_8N1, PI_RX_PIN, PI_TX_PIN);  // Pi comms
    initIMU();
    initMotorPins();
    initEncoders();
    Serial.println("System initialized.");
}

void loop() {
    mpu6050.update();
    mpu_angle = mpu6050.getAngleZ() * PI / 180.0;
    while (mpu_angle > PI) mpu_angle -= 2 * PI;
    while (mpu_angle < -PI) mpu_angle += 2 * PI;

    // Read (v, w) commands from Pi
    readPiCommand();

    unsigned long current_time = millis();
    if (current_time - last_time >= sample_time) {
        Odometry();

        // Watchdog: stop if Pi stops sending commands
        if (current_time - last_cmd_time > CMD_TIMEOUT_MS) {
            cmd_v = 0.0;
            cmd_w = 0.0;
        }

        // Convert (v, w) -> target wheel velocities
        target_left_velocity  = cmd_v - cmd_w * (WHEEL_BASE / 2.0);
        target_right_velocity = cmd_v + cmd_w * (WHEEL_BASE / 2.0);
        target_left_velocity  = constrain(target_left_velocity,  -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
        target_right_velocity = constrain(target_right_velocity, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);

        wheelVelocityControl();
        sendState();

        last_time = current_time;
    }
}

void Odometry(){
    long current_left_count = left_encoder_count;
    long current_right_count = right_encoder_count;

    long delta_left = current_left_count - last_left_count;
    long delta_right = current_right_count - last_right_count;

    last_left_count = current_left_count;
    last_right_count = current_right_count;

    float dt = sample_time / 1000.0;  // seconds

    left_distance = (delta_left / ENCODER_TICKS_PER_REV) * (2 * PI * WHEEL_RADIUS);
    right_distance = (delta_right / ENCODER_TICKS_PER_REV) * (2 * PI * WHEEL_RADIUS);

    left_velocity = left_distance / dt;
    right_velocity = right_distance / dt;

    float delta_theta = (right_distance - left_distance) / WHEEL_BASE;

    float delta_x = ((left_distance + right_distance) / 2) * cos(theta + delta_theta / 2.0);
    float delta_y = ((left_distance + right_distance) / 2) * sin(theta + delta_theta / 2.0);

    x += delta_x;
    y += delta_y;
    
    theta += delta_theta;

    while (theta > PI) theta -= 2 * PI;
    while (theta < -PI) theta += 2 * PI;
}

// Read one line from Pi: "v:<float>,w:<float>\n"
void readPiCommand() {
    static char buf[32];
    static uint8_t idx = 0;
    while (Serial2.available()) {
        char c = Serial2.read();
        if (c == '\n') {
            buf[idx] = '\0';
            // Parse v:<val>,w:<val>
            char *v_ptr = strstr(buf, "v:");
            char *w_ptr = strstr(buf, ",w:");
            if (v_ptr && w_ptr) {
                cmd_v = atof(v_ptr + 2);
                cmd_w = atof(w_ptr + 3);
                last_cmd_time = millis();
            }
            idx = 0;
        } else if (idx < sizeof(buf) - 1) {
            buf[idx++] = c;
        }
    }
}

// Send robot state to Pi: "x,y,theta,lv,rv\n"
void sendState() {
    Serial2.print(x, 3);            Serial2.print(',');
    Serial2.print(y, 3);            Serial2.print(',');
    Serial2.print(theta, 4);        Serial2.print(',');
    Serial2.print(mpu_angle, 4);    Serial2.print(',');
    Serial2.print(left_velocity, 3); Serial2.print(',');
    Serial2.print(right_velocity, 3);
    Serial2.println();
}

void wheelVelocityControl(){
    float dt = sample_time / 1000.0f;

    float left_error        = target_left_velocity - left_velocity;
    left_error_sum         += left_error * dt;
    float left_error_deriv  = (left_error - left_last_error) / dt;
    float left_pwm          = Kp_left * left_error
                            + Ki_left * left_error_sum
                            + Kd_left * left_error_deriv;
    left_last_error = left_error;

    float right_error        = target_right_velocity - right_velocity;
    right_error_sum         += right_error * dt;
    float right_error_deriv  = (right_error - right_last_error) / dt;
    float right_pwm          = Kp_right * right_error
                             + Ki_right * right_error_sum
                             + Kd_right * right_error_deriv;
    right_last_error = right_error;

    setMotor((int)constrain(left_pwm,  -MAX_PWM, MAX_PWM),
             (int)constrain(right_pwm, -MAX_PWM, MAX_PWM));
}