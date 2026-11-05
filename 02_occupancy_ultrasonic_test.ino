/*
  Test: Cabin occupancy ultrasonic sensor
  Keeps the final project's pins:
    TRIG = D5
    ECHO = D6
  The current controller considers an object <= 8 cm away as detected/occupied.

  Open Serial Monitor at 9600 baud.
*/

const byte CABIN_TRIG_PIN = 5;
const byte CABIN_ECHO_PIN = 6;
const int OCCUPIED_THRESHOLD_CM = 8;
const unsigned long ECHO_TIMEOUT_US = 12000UL;

void setup() {
  Serial.begin(9600);
  pinMode(CABIN_TRIG_PIN, OUTPUT);
  pinMode(CABIN_ECHO_PIN, INPUT);
  digitalWrite(CABIN_TRIG_PIN, LOW);

  Serial.println(F("Cabin-occupancy ultrasonic sensor test"));
  Serial.println(F("Pins: TRIG D5, ECHO D6; threshold <= 8 cm"));
  Serial.println(F("Place/remove an object within the sensor's field of view."));
}

void loop() {
  digitalWrite(CABIN_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(CABIN_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(CABIN_TRIG_PIN, LOW);

  unsigned long duration = pulseIn(CABIN_ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) {
    Serial.println(F("No echo: check power, wiring, sensor alignment, and target."));
  } else {
    float distanceCm = (duration * 0.0343f) / 2.0f;
    Serial.print(F("Distance: "));
    Serial.print(distanceCm, 1);
    Serial.print(F(" cm | Status: "));
    Serial.println(distanceCm <= OCCUPIED_THRESHOLD_CM ? F("OBJECT DETECTED (occupied threshold)") : F("CLEAR"));
  }

  delay(300);
}
