#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// --- Base Car Pins (Movement) ---
const int BASE_IN1 = 18;
const int BASE_IN2 = 19;
const int BASE_IN3 = 25;
const int BASE_IN4 = 26;

// --- Reaction Wheel Pins (Balancing) ---
const int RW_IN1 = 4;
const int RW_IN2 = 5;

const int RW_PWM_MIN = 70;
const int RW_PWM_MAX = 255;

// --- PID Tuning ---
float Kp = 300.0;
float Ki =   5.0;
float Kd =   4.0;

const float INTEGRAL_LIMIT = 50.0;
const float FALL_THRESHOLD = 100.0;
float targetAngle = 85.0;

float integral      = 0.0;
float previousError = 0.0;
unsigned long lastTime = 0;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5);

  pinMode(BASE_IN1, OUTPUT);
  pinMode(BASE_IN2, OUTPUT);
  pinMode(BASE_IN3, OUTPUT);
  pinMode(BASE_IN4, OUTPUT);
  stopBaseCar();

  pinMode(RW_IN1, OUTPUT);
  pinMode(RW_IN2, OUTPUT);
  digitalWrite(RW_IN1, LOW);
  digitalWrite(RW_IN2, LOW);

  if (!bno.begin()) {
    Serial.println("BNO055 not found! Check wiring.");
    while (1);
  }
  bno.setExtCrystalUse(true);
  delay(1000);

  Serial.println("System ready.");
  lastTime = millis();
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0f;
  lastTime = now;
  if (dt <= 0) dt = 0.01f;

  // --- Base car drive (full speed forward) ---
  analogWrite(BASE_IN2, 255);
  digitalWrite(BASE_IN1, LOW);
  analogWrite(BASE_IN3, 255);
  digitalWrite(BASE_IN4, LOW);

  // --- IMU read ---
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  float currentAngle = orientationData.orientation.y;
  float side         = orientationData.orientation.z;
  float heading      = orientationData.orientation.x;

  float error = targetAngle - currentAngle;
  if (side < 0) error *= -1;

  // Fall detection
  if (abs(error) > FALL_THRESHOLD) {
    driveReactionWheel(0);
    integral = 0;
    previousError = 0;
    Serial.print("[FELL] angle="); Serial.print(currentAngle);
    Serial.print(" | side=");      Serial.print(side);
    Serial.print(" | error=");     Serial.println(error);
    delay(100);
    return;
  }

  // --- PID ---
  integral += error * dt;
  integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  float derivative = (error - previousError) / dt;

  if (abs(integral) > INTEGRAL_LIMIT - 0.1) {
    integral   = 0;
    derivative = 0;
    error      = 0;
  }

  previousError = error;
  float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

  driveReactionWheel(output);

  // --- Log ---
  Serial.print("[IMU] heading="); Serial.print(heading);
  Serial.print(" | angle=");      Serial.print(currentAngle);
  Serial.print(" | side=");       Serial.print(side);
  Serial.print(" | error=");      Serial.print(error);
  Serial.print(" | intg=");       Serial.print(integral);
  Serial.print(" | deriv=");      Serial.print(derivative);
  Serial.print(" | output=");     Serial.println(output);

  delay(10); // 100 Hz
}

void driveReactionWheel(float effort) {
  effort = constrain(effort, -50000.0f, 50000.0f);

  if (abs(effort) < 10) {
    digitalWrite(RW_IN1, LOW);
    digitalWrite(RW_IN2, LOW);
    return;
  }

  int pwmVal = map(abs(effort), 10, 3000, RW_PWM_MIN, RW_PWM_MAX);
  pwmVal = constrain(pwmVal, RW_PWM_MIN, RW_PWM_MAX);

  if (effort > 0) {
    analogWrite(RW_IN1, pwmVal);
    digitalWrite(RW_IN2, LOW);
  } else {
    digitalWrite(RW_IN1, LOW);
    analogWrite(RW_IN2, pwmVal);
  }
}

void stopBaseCar() {
  digitalWrite(BASE_IN1, LOW);
  digitalWrite(BASE_IN2, LOW);
  digitalWrite(BASE_IN3, LOW);
  digitalWrite(BASE_IN4, LOW);
}
