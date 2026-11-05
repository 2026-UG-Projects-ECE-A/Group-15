/*
  Test: Lift-height ultrasonic sensor
  Keeps the final project's pins:
    TRIG = D3
    ECHO = D4
  Floor target distances are explicit and uneven: 2, 20, and 40 cm.
  Tolerance matches the current controller: +/- 2 cm.

  Open Serial Monitor at 9600 baud.
*/

const byte HEIGHT_TRIG_PIN = 3;
const byte HEIGHT_ECHO_PIN = 4;

const int FLOOR_1_DISTANCE_CM = 2;
const int FLOOR_2_DISTANCE_CM = 20;
const int FLOOR_3_DISTANCE_CM = 40;
const int FLOOR_TOLERANCE_CM = 2;
const unsigned long ECHO_TIMEOUT_US = 25000UL;

int measureDistanceCm() {
  digitalWrite(HEIGHT_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(HEIGHT_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(HEIGHT_TRIG_PIN, LOW);

  unsigned long duration = pulseIn(HEIGHT_ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) return -1;

  float distanceCm = (duration * 0.0343f) / 2.0f;
  return (int)(distanceCm + 0.5f); // round to nearest cm, like main controller
}

int identifyFloor(int distanceCm) {
  if (distanceCm < 0) return 0;
  if (abs(distanceCm - FLOOR_1_DISTANCE_CM) <= FLOOR_TOLERANCE_CM) return 1;
  if (abs(distanceCm - FLOOR_2_DISTANCE_CM) <= FLOOR_TOLERANCE_CM) return 2;
  if (abs(distanceCm - FLOOR_3_DISTANCE_CM) <= FLOOR_TOLERANCE_CM) return 3;
  return 0;
}

void setup() {
  Serial.begin(9600);
  pinMode(HEIGHT_TRIG_PIN, OUTPUT);
  pinMode(HEIGHT_ECHO_PIN, INPUT);
  digitalWrite(HEIGHT_TRIG_PIN, LOW);

  Serial.println(F("Lift-height ultrasonic sensor test"));
  Serial.println(F("Pins: TRIG D3, ECHO D4"));
  Serial.println(F("Calibrated distances: F1=2 cm, F2=20 cm, F3=40 cm; tolerance +/-2 cm"));
  Serial.println(F("Move the lift/target by hand and observe the readings."));
}

void loop() {
  int distanceCm = measureDistanceCm();

  if (distanceCm < 0) {
    Serial.println(F("No echo: check sensor power, TRIG/ECHO wiring, target, and sensor alignment."));
  } else {
    Serial.print(F("Distance: "));
    Serial.print(distanceCm);
    Serial.print(F(" cm | Classification: "));

    int floorNumber = identifyFloor(distanceCm);
    if (floorNumber == 0) {
      Serial.println(F("BETWEEN FLOORS / not in a calibrated range"));
    } else {
      Serial.print(F("FLOOR "));
      Serial.println(floorNumber);
    }
  }

  delay(250);
}
