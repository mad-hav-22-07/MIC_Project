/*
 * Motor PWM Test
 * ---------------
 * Type a value from -255 to 255 in the Serial Monitor to test both drive motors.
 * Positive = forward, negative = reverse, 0 = stop.
 * Use this to verify motor wiring and direction before running the PID sketch.
 */

const int AIN1 = 4;
const int AIN2 = 5;
const int AIN3 = 25;
const int AIN4 = 26;

void setup() {
  Serial.begin(115200);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(AIN3, OUTPUT);
  pinMode(AIN4, OUTPUT);
  stopMotors();
  Serial.println("Motor PWM Test ready. Type -255 to 255 and press Enter.");
}

void loop() {
  if (Serial.available()) {
    int input = Serial.parseInt();
    while (Serial.available()) Serial.read(); // flush

    input = constrain(input, -255, 255);

    if (input > 0) {
      analogWrite(AIN2, input);
      digitalWrite(AIN1, LOW);
      analogWrite(AIN3, input);
      digitalWrite(AIN4, LOW);
      Serial.print("Forward | PWM = "); Serial.println(input);

    } else if (input < 0) {
      digitalWrite(AIN2, LOW);
      analogWrite(AIN1, abs(input));
      digitalWrite(AIN3, LOW);
      analogWrite(AIN4, abs(input));
      Serial.print("Reverse | PWM = "); Serial.println(abs(input));

    } else {
      stopMotors();
      Serial.println("Stopped");
    }
  }
}

void stopMotors() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(AIN3, LOW);
  digitalWrite(AIN4, LOW);
}
