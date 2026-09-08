/*
  SBR-X Firmware
  Self-Balancing Robot with Line Following and Obstacle Avoidance

  Board: Arduino Uno / Nano
  Sensors: MPU6050 (I2C), dual ultrasonic (HC-SR04 style), 4-channel IR array
  Driver: L298N / TB6612FNG style (IN1-IN4 direction, ENA/ENB PWM speed)

  Pin map (update to match your actual wiring):
    MPU6050        SDA -> A4   SCL -> A5
    Ultrasonic L   Trig -> D2  Echo -> D3
    Ultrasonic R   Trig -> D4  Echo -> D5
    IR array       A0, A1, A2, A3 (left to right)
    Motor driver   IN1 -> D6   IN2 -> D7   IN3 -> D8   IN4 -> D9
                   ENA -> D10 (left speed, PWM)   ENB -> D11 (right speed, PWM)
    Mode button    D12 (INPUT_PULLUP, active LOW)

  Modes (cycled by the mode button):
    0 = Balance            (default, always active underneath the others)
    1 = Line Following
    2 = Obstacle Avoidance

  This sketch has no external library dependencies beyond Wire.h, which
  ships with the Arduino IDE.
*/

#include <Wire.h>

// ---------------------------------------------------------------
// Pin definitions
// ---------------------------------------------------------------
const uint8_t PIN_TRIG_L   = 2;
const uint8_t PIN_ECHO_L   = 3;
const uint8_t PIN_TRIG_R   = 4;
const uint8_t PIN_ECHO_R   = 5;

const uint8_t PIN_IN1      = 6;
const uint8_t PIN_IN2      = 7;
const uint8_t PIN_IN3      = 8;
const uint8_t PIN_IN4      = 9;
const uint8_t PIN_ENA      = 10;  // left motor speed (PWM)
const uint8_t PIN_ENB      = 11;  // right motor speed (PWM)

const uint8_t PIN_BUTTON   = 12;

const uint8_t PIN_IR[4]    = { A0, A1, A2, A3 };

const uint8_t MPU_ADDR     = 0x68;

// ---------------------------------------------------------------
// Tunable constants (start here when calibrating the robot)
// ---------------------------------------------------------------
float KP = 18.0;
float KI = 140.0;
float KD = 0.6;

const float SAFE_TILT_LIMIT   = 40.0;   // degrees; beyond this, motors cut out
const float COMPLEMENTARY_A   = 0.98;   // gyro vs accelerometer trust
const float LOOP_DT           = 0.005;  // seconds (200 Hz loop)

const int   LINE_THRESHOLD    = 500;    // analogRead value that counts as "on the line"
const float LINE_BIAS_GAIN    = 6.0;    // degrees of lean-angle bias per unit of line error

const float OBSTACLE_DISTANCE_CM = 20.0;
const float OBSTACLE_BIAS_GAIN   = 4.0;

const int   MOTOR_MAX_PWM     = 255;
const int   MOTOR_MIN_PWM     = 60;     // overcome static friction

// ---------------------------------------------------------------
// State
// ---------------------------------------------------------------
enum Mode { BALANCE = 0, LINE_FOLLOW = 1, OBSTACLE_AVOID = 2 };
Mode currentMode = BALANCE;

float filteredAngle = 0.0;
float pidIntegral   = 0.0;
float lastError     = 0.0;

bool  lastButtonState = HIGH;
unsigned long lastButtonChangeMs = 0;
const unsigned long DEBOUNCE_MS = 40;

unsigned long lastLoopMicros = 0;

// ---------------------------------------------------------------
// Setup
// ---------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(PIN_TRIG_L, OUTPUT);
  pinMode(PIN_ECHO_L, INPUT);
  pinMode(PIN_TRIG_R, OUTPUT);
  pinMode(PIN_ECHO_R, INPUT);

  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);

  pinMode(PIN_BUTTON, INPUT_PULLUP);

  mpuInit();

  // Let the filter settle on the initial angle instead of starting at 0.
  for (int i = 0; i < 200; i++) {
    updateAngleEstimate();
    delay(2);
  }

  lastLoopMicros = micros();
}

// ---------------------------------------------------------------
// Main loop
// ---------------------------------------------------------------
void loop() {
  unsigned long nowMicros = micros();
  float dt = (nowMicros - lastLoopMicros) / 1000000.0;
  if (dt < LOOP_DT) return;   // hold a fixed-rate loop
  lastLoopMicros = nowMicros;

  handleModeButton();
  updateAngleEstimate();

  if (fabs(filteredAngle) > SAFE_TILT_LIMIT) {
    stopMotors();
    pidIntegral = 0.0;   // avoid integral windup while stopped
    return;
  }

  float bias = computeModeBias();
  float output = computeBalancePID(bias, dt);
  driveFromBalanceOutput(output);
}

// =================================================================
// Balance: sensor fusion + PID
// =================================================================
void updateAngleEstimate() {
  int16_t ax, ay, az, gx, gy, gz;
  mpuReadRaw(ax, ay, az, gx, gy, gz);

  float accelAngle = atan2((float)ay, (float)az) * 180.0 / PI;
  float gyroRate    = gx / 131.0;  // deg/s, +-250 dps sensitivity

  filteredAngle = COMPLEMENTARY_A * (filteredAngle + gyroRate * LOOP_DT)
                  + (1.0 - COMPLEMENTARY_A) * accelAngle;
}

float computeBalancePID(float targetBias, float dt) {
  float target = targetBias;   // 0 degrees, plus the active mode's bias
  float error = target - filteredAngle;

  pidIntegral += error * dt;
  pidIntegral = constrain(pidIntegral, -50.0, 50.0);  // anti-windup clamp

  float derivative = (error - lastError) / dt;
  lastError = error;

  return KP * error + KI * pidIntegral + KD * derivative;
}

void driveFromBalanceOutput(float output) {
  int pwm = (int)constrain(fabs(output), 0, MOTOR_MAX_PWM);
  if (pwm > 0 && pwm < MOTOR_MIN_PWM) pwm = MOTOR_MIN_PWM;

  bool forward = output >= 0;
  setMotor(true,  forward, pwm);
  setMotor(false, forward, pwm);
}

// =================================================================
// Mode logic: only ever biases the PID target, never drives motors directly
// =================================================================
float computeModeBias() {
  switch (currentMode) {
    case LINE_FOLLOW:    return lineFollowBias();
    case OBSTACLE_AVOID: return obstacleAvoidBias();
    case BALANCE:
    default:             return 0.0;
  }
}

float lineFollowBias() {
  int reading[4];
  for (int i = 0; i < 4; i++) reading[i] = analogRead(PIN_IR[i]);

  // Weighted position: negative = line is to the left, positive = to the right
  float weights[4] = { -1.5, -0.5, 0.5, 1.5 };
  float weightedSum = 0.0;
  int   activeCount  = 0;

  for (int i = 0; i < 4; i++) {
    if (reading[i] > LINE_THRESHOLD) {
      weightedSum += weights[i];
      activeCount++;
    }
  }

  if (activeCount == 0) return 0.0;  // line lost, hold last balance target
  float position = weightedSum / activeCount;
  return position * LINE_BIAS_GAIN;
}

float obstacleAvoidBias() {
  float distL = readUltrasonicCm(PIN_TRIG_L, PIN_ECHO_L);
  float distR = readUltrasonicCm(PIN_TRIG_R, PIN_ECHO_R);

  bool blockedL = distL > 0 && distL < OBSTACLE_DISTANCE_CM;
  bool blockedR = distR > 0 && distR < OBSTACLE_DISTANCE_CM;

  if (blockedL && !blockedR) return  OBSTACLE_BIAS_GAIN;  // steer right
  if (blockedR && !blockedL) return -OBSTACLE_BIAS_GAIN;  // steer left
  return 0.0;
}

// =================================================================
// Mode button (debounced)
// =================================================================
void handleModeButton() {
  bool reading = digitalRead(PIN_BUTTON);
  unsigned long nowMs = millis();

  if (reading != lastButtonState) {
    lastButtonChangeMs = nowMs;
  }

  if ((nowMs - lastButtonChangeMs) > DEBOUNCE_MS) {
    if (reading == LOW && lastButtonState == HIGH) {
      currentMode = (Mode)((currentMode + 1) % 3);
      pidIntegral = 0.0;
    }
  }
  lastButtonState = reading;
}

// =================================================================
// Motor driver
// =================================================================
void setMotor(bool isLeft, bool forward, int pwm) {
  if (isLeft) {
    digitalWrite(PIN_IN1, forward ? HIGH : LOW);
    digitalWrite(PIN_IN2, forward ? LOW  : HIGH);
    analogWrite(PIN_ENA, pwm);
  } else {
    digitalWrite(PIN_IN3, forward ? HIGH : LOW);
    digitalWrite(PIN_IN4, forward ? LOW  : HIGH);
    analogWrite(PIN_ENB, pwm);
  }
}

void stopMotors() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
  analogWrite(PIN_ENA, 0);
  analogWrite(PIN_ENB, 0);
}

// =================================================================
// Ultrasonic distance (HC-SR04 style)
// =================================================================
float readUltrasonicCm(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, 20000UL);  // 20 ms timeout ~3.4 m
  if (duration == 0) return -1.0;  // no echo, treat as "no obstacle"
  return duration / 58.0;          // microseconds to centimeters
}

// =================================================================
// MPU6050 (raw register access, no external library)
// =================================================================
void mpuInit() {
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1
  Wire.write(0x00);  // wake the sensor up
  Wire.endTransmission(true);
}

void mpuReadRaw(int16_t &ax, int16_t &ay, int16_t &az,
                int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // starting register: ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();  // discard temperature
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
}
