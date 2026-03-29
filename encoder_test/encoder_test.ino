// Encoder Calibration Test for ESP32
// This code helps you determine the correct ENCODER_TICKS_PER_REV value

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

volatile long right_encoder_count = 0;
volatile long left_encoder_count = 0;

const int TEST_PWM = 150;  // Moderate speed for testing
const float WHEEL_RADIUS = 4.2;  // cm

void IRAM_ATTR right_encoder_ISR() {
    if (digitalRead(ENCODER_RIGHT_B)) right_encoder_count--;
    else right_encoder_count++;
}

void IRAM_ATTR left_encoder_ISR() {
    if (digitalRead(ENCODER_LEFT_B)) left_encoder_count++;
    else left_encoder_count--;
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
    
    analogWrite(LEFT_SPD_PIN, abs(leftPWM));
    analogWrite(RIGHT_SPD_PIN, abs(rightPWM));
}

void stopMotors() {
    setMotor(0, 0);
}

void setup() {
    Serial.begin(115200);
    initMotorPins();
    initEncoders();
    
    delay(2000);
    
    Serial.println("\n\n========================================");
    Serial.println("MANUAL WHEEL ROTATION TEST");
    Serial.println("========================================");
    Serial.println("EASIEST CALIBRATION METHOD:");
    Serial.println("");
    Serial.println("1. Lift robot so ONE wheel is off ground");
    Serial.println("2. Manually rotate that wheel SLOWLY");
    Serial.println("3. Count exactly 10 full rotations");
    Serial.println("4. Read ticks from Serial Monitor");
    Serial.println("5. ENCODER_TICKS_PER_REV = Ticks / 10");
    Serial.println("");
    Serial.println("TIP: Mark wheel with tape to count rotations");
    Serial.println("========================================");
    Serial.println("");
    Serial.println("Encoders are now active.");
    Serial.println("Rotate wheels and watch counts below:");
    Serial.println("");
}

void loop() {
    // Print encoder values continuously
    static unsigned long last_print = 0;
    if (millis() - last_print > 200) {
        Serial.print("Left: ");
        Serial.print(left_encoder_count);
        Serial.print("  |  Right: ");
        Serial.println(right_encoder_count);
        last_print = millis();
    }
}
