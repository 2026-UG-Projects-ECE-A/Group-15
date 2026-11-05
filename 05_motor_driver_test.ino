/*
  Test: Motor driver direction outputs
  Keeps the final project's pins:
    UP = D12, DOWN = D13

  Open Serial Monitor at 9600 baud and send:
    u = brief UP pulse
    d = brief DOWN pulse
    s = stop immediately

  Each direction pulse automatically stops after 700 ms. Only one direction
  output is ever HIGH at once. Test with the lift unloaded/mechanically supported.
  This assumes active-HIGH motor-driver inputs, matching the current project.
*/

const byte MOTOR_UP_PIN = 12;
const byte MOTOR_DOWN_PIN = 13;
const unsigned long TEST_PULSE_MS = 700UL;

bool motorPulseActive = false;
unsigned long motorPulseStartMs = 0;

void stopMotor() {
  digitalWrite(MOTOR_UP_PIN, LOW);
  digitalWrite(MOTOR_DOWN_PIN, LOW);
  motorPulseActive = false;
}

void startPulse(bool goUp) {
  if (motorPulseActive) {
    Serial.println(F("A test pulse is already active. Send 's' to stop; otherwise wait for auto-stop."));
    return;
  }

  // Force both outputs OFF before selecting a direction.
  stopMotor();
  delay(100);

  if (goUp) {
    digitalWrite(MOTOR_DOWN_PIN, LOW);
    digitalWrite(MOTOR_UP_PIN, HIGH);
    Serial.println(F("UP output active for at most 700 ms."));
  } else {
    digitalWrite(MOTOR_UP_PIN, LOW);
    digitalWrite(MOTOR_DOWN_PIN, HIGH);
    Serial.println(F("DOWN output active for at most 700 ms."));
  }

  motorPulseActive = true;
  motorPulseStartMs = millis();
}

void setup() {
  Serial.begin(9600);
  pinMode(MOTOR_UP_PIN, OUTPUT);
  pinMode(MOTOR_DOWN_PIN, OUTPUT);
  stopMotor();

  Serial.println(F("Motor-driver test: UP D12, DOWN D13"));
  Serial.println(F("Send u=UP pulse, d=DOWN pulse, s=stop."));
  Serial.println(F("Outputs start OFF; each pulse auto-stops after 700 ms."));
}

void loop() {
  while (Serial.available() > 0) {
    char command = Serial.read();

    if (command == 's' || command == 'S') {
      stopMotor();
      Serial.println(F("STOP: both motor outputs LOW."));
    } else if (command == 'u' || command == 'U') {
      startPulse(true);
    } else if (command == 'd' || command == 'D') {
      startPulse(false);
    }
    // Ignore carriage return/newline from Serial Monitor.
  }

  if (motorPulseActive && millis() - motorPulseStartMs >= TEST_PULSE_MS) {
    stopMotor();
    Serial.println(F("Pulse complete: both motor outputs LOW."));
  }
}
