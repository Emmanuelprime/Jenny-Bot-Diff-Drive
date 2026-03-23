#include <PID_v1.h>

// Motor Driver Connection PINs (ESP32)
#define LEFT_SPD_PIN 16   // PWM
#define LEFT_DIR1_PIN 17  // Dir
#define LEFT_DIR2_PIN 18  // Dir
#define RIGHT_SPD_PIN 25  // PWM
#define RIGHT_DIR1_PIN 26 // Dir
#define RIGHT_DIR2_PIN 27 // Dir

// Wheel Encoders Connection PINs (ESP32)
#define right_encoder_phaseA 34  // Interrupt 
#define right_encoder_phaseB 35  
#define left_encoder_phaseA 32   // Interrupt
#define left_encoder_phaseB 33

// Encoders
unsigned int right_encoder_counter = 0;
unsigned int left_encoder_counter = 0;
String right_wheel_sign = "p";  // 'p' = positive, 'n' = negative
String left_wheel_sign = "p";  // 'p' = positive, 'n' = negative
unsigned long last_millis = 0;
const unsigned long interval = 100;

// Interpret Serial Messages
bool is_right_wheel_cmd = false;
bool is_left_wheel_cmd = false;
bool is_right_wheel_forward = true;
bool is_left_wheel_forward = true;
char value[] = "00.00";
uint8_t value_idx = 0;
bool is_cmd_complete = false;

// PID
// Setpoint - Desired
double right_wheel_cmd_vel = 0.0;     // rad/s
double left_wheel_cmd_vel = 0.0;      // rad/s
// Input - Measurement
double right_wheel_meas_vel = 0.0;    // rad/s
double left_wheel_meas_vel = 0.0;     // rad/s
// Output - Command
double right_wheel_cmd = 0.0;             // 0-255
double left_wheel_cmd = 0.0;              // 0-255
// Tuning
double Kp_r = 11.5;
double Ki_r = 7.5;
double Kd_r = 0.1;
double Kp_l = 12.8;
double Ki_l = 8.3;
double Kd_l = 0.1;
// Controller
PID rightMotor(&right_wheel_meas_vel, &right_wheel_cmd, &right_wheel_cmd_vel, Kp_r, Ki_r, Kd_r, DIRECT);
PID leftMotor(&left_wheel_meas_vel, &left_wheel_cmd, &left_wheel_cmd_vel, Kp_l, Ki_l, Kd_l, DIRECT);

void setup() {
  // Init Motor Driver Connection PINs
  pinMode(LEFT_SPD_PIN, OUTPUT);
  pinMode(LEFT_DIR1_PIN, OUTPUT);
  pinMode(LEFT_DIR2_PIN, OUTPUT);
  pinMode(RIGHT_SPD_PIN, OUTPUT);
  pinMode(RIGHT_DIR1_PIN, OUTPUT);
  pinMode(RIGHT_DIR2_PIN, OUTPUT);

  // Set Motor Rotation Direction (Initial forward)
  digitalWrite(LEFT_DIR1_PIN, LOW);
  digitalWrite(LEFT_DIR2_PIN, HIGH);
  digitalWrite(RIGHT_DIR1_PIN, LOW);
  digitalWrite(RIGHT_DIR2_PIN, HIGH);

  rightMotor.SetMode(AUTOMATIC);
  leftMotor.SetMode(AUTOMATIC);
  Serial.begin(115200);

  // Init encoders
  pinMode(right_encoder_phaseA, INPUT_PULLUP);
  pinMode(right_encoder_phaseB, INPUT_PULLUP);
  pinMode(left_encoder_phaseA, INPUT_PULLUP);
  pinMode(left_encoder_phaseB, INPUT_PULLUP);
  // Set Callback for Wheel Encoders Pulse
  attachInterrupt(digitalPinToInterrupt(right_encoder_phaseA), rightEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(left_encoder_phaseA), leftEncoderCallback, RISING);
}

void loop() {
  // Read and Interpret Wheel Velocity Commands
  if (Serial.available())
  {
    char chr = Serial.read();
    // Right Wheel Motor
    if(chr == 'r')
    {
      is_right_wheel_cmd = true;
      is_left_wheel_cmd = false;
      value_idx = 0;
      is_cmd_complete = false;
    }
    // Left Wheel Mo tor
    else if(chr == 'l')
    {
      is_right_wheel_cmd = false;
      is_left_wheel_cmd = true;
      value_idx = 0;
    }
    // Positive direction
    else if(chr == 'p')
    {
      if(is_right_wheel_cmd && !is_right_wheel_forward)
      {
        // change the direction of the rotation
        digitalWrite(RIGHT_DIR1_PIN, HIGH - digitalRead(RIGHT_DIR1_PIN));
        digitalWrite(RIGHT_DIR2_PIN, HIGH - digitalRead(RIGHT_DIR2_PIN));
        is_right_wheel_forward = true;
      }
      else if(is_left_wheel_cmd && !is_left_wheel_forward)
      {
        // change the direction of the rotation
        digitalWrite(LEFT_DIR1_PIN, HIGH - digitalRead(LEFT_DIR1_PIN));
        digitalWrite(LEFT_DIR2_PIN, HIGH - digitalRead(LEFT_DIR2_PIN));
        is_left_wheel_forward = true;
      }
    }
    // Negative direction
    else if(chr == 'n')
    {
      if(is_right_wheel_cmd && is_right_wheel_forward)
      {
        // change the direction of the rotation
        digitalWrite(RIGHT_DIR1_PIN, HIGH - digitalRead(RIGHT_DIR1_PIN));
        digitalWrite(RIGHT_DIR2_PIN, HIGH - digitalRead(RIGHT_DIR2_PIN));
        is_right_wheel_forward = false;
      }
      else if(is_left_wheel_cmd && is_left_wheel_forward)
      {
        // change the direction of the rotation
        digitalWrite(LEFT_DIR1_PIN, HIGH - digitalRead(LEFT_DIR1_PIN));
        digitalWrite(LEFT_DIR2_PIN, HIGH - digitalRead(LEFT_DIR2_PIN));
        is_left_wheel_forward = false;
      }
    }
    // Separator
    else if(chr == ',')
    {
      if(is_right_wheel_cmd)
      {
        right_wheel_cmd_vel = atof(value);
      }
      else if(is_left_wheel_cmd)
      {
        left_wheel_cmd_vel = atof(value);
        is_cmd_complete = true;
      }
      // Reset for next command
      value_idx = 0;
      value[0] = '0';
      value[1] = '0';
      value[2] = '.';
      value[3] = '0';
      value[4] = '0';
      value[5] = '\0';
    }
    // Command Value
    else
    {
      if(value_idx < 5)
      {
        value[value_idx] = chr;
        value_idx++;
      }
    }
  }

  // Encoder
  unsigned long current_millis = millis();
  if(current_millis - last_millis >= interval)
  {
    // Convert encoder counts to rad/s (207 ticks per revolution)
    right_wheel_meas_vel = (10 * right_encoder_counter * (60.0/207.0)) * 0.10472;
    left_wheel_meas_vel = (10 * left_encoder_counter * (60.0/207.0)) * 0.10472;
    
    rightMotor.Compute();
    leftMotor.Compute();

    // Ignore commands smaller than inertia
    if(right_wheel_cmd_vel == 0.0)
    {
      right_wheel_cmd = 0.0;
    }
    if(left_wheel_cmd_vel == 0.0)
    {
      left_wheel_cmd = 0.0;
    }

    String encoder_read = "r" + right_wheel_sign + String(right_wheel_meas_vel) + ",l" + left_wheel_sign + String(left_wheel_meas_vel) + ",";
    Serial.println(encoder_read);
    last_millis = current_millis;
    right_encoder_counter = 0;
    left_encoder_counter = 0;

    analogWrite(RIGHT_SPD_PIN, right_wheel_cmd);
    analogWrite(LEFT_SPD_PIN, left_wheel_cmd);
  }
}

// New pulse from Right Wheel Encoder
void IRAM_ATTR rightEncoderCallback()
{
  if(digitalRead(right_encoder_phaseB) == HIGH)
  {
    right_wheel_sign = "n";  // Reversed direction
  }
  else
  {
    right_wheel_sign = "p";
  }
  right_encoder_counter++;
}

// New pulse from Left Wheel Encoder
void IRAM_ATTR leftEncoderCallback()
{
  if(digitalRead(left_encoder_phaseB) == HIGH)
  {
    left_wheel_sign = "p";
  }
  else
  {
    left_wheel_sign = "n";  // Reversed direction
  }
  left_encoder_counter++;
}