/*
  Test: 7-segment display through a 74HC595 shift register
  Keeps the final project's pins:
    DATA = D7, LATCH = D8, CLOCK = D9
  Patterns are from the existing project and assume a common-anode display.

  Open Serial Monitor at 9600 baud. It cycles through 0, 1, 2, 3.
*/

const byte DISPLAY_DATA_PIN = 7;
const byte DISPLAY_LATCH_PIN = 8;
const byte DISPLAY_CLOCK_PIN = 9;

const byte DISPLAY_NUM[] = {
  0b11000000, // 0
  0b11111001, // 1
  0b10100100, // 2
  0b10110000  // 3
};

void showDigit(byte digit) {
  digitalWrite(DISPLAY_LATCH_PIN, LOW);
  shiftOut(DISPLAY_DATA_PIN, DISPLAY_CLOCK_PIN, MSBFIRST, DISPLAY_NUM[digit]);
  digitalWrite(DISPLAY_LATCH_PIN, HIGH);
}

void setup() {
  Serial.begin(9600);
  pinMode(DISPLAY_DATA_PIN, OUTPUT);
  pinMode(DISPLAY_LATCH_PIN, OUTPUT);
  pinMode(DISPLAY_CLOCK_PIN, OUTPUT);

  Serial.println(F("7-segment display test: pins DATA D7, LATCH D8, CLOCK D9"));
  Serial.println(F("Expected: 0 -> 1 -> 2 -> 3, repeating."));
}

void loop() {
  for (byte digit = 0; digit <= 3; digit++) {
    showDigit(digit);
    Serial.print(F("Displaying digit: "));
    Serial.println(digit);
    delay(1000);
  }
}
