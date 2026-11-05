/*
  Test: All six lift buttons
  Keeps the final project's pins:
    Outside call buttons: A0, A1, A2
    Inside destination buttons: A3, A4, A5

  Matches the current controller's assumption: INPUT with external pull-down
  resistors, so released = LOW and pressed = HIGH. Do not use this sketch as-is
  if your wiring instead uses INPUT_PULLUP (pressed = LOW).

  Open Serial Monitor at 9600 baud and press/release each button.
*/

const byte BUTTON_PINS[6] = {A0, A1, A2, A3, A4, A5};
const char *BUTTON_NAMES[6] = {
  "Outside call - Floor 1 (A0)",
  "Outside call - Floor 2 (A1)",
  "Outside call - Floor 3 (A2)",
  "Inside select - Floor 1 (A3)",
  "Inside select - Floor 2 (A4)",
  "Inside select - Floor 3 (A5)"
};

const unsigned long DEBOUNCE_MS = 35;
bool lastRaw[6];
bool stableState[6];
unsigned long lastChangeMs[6];

void setup() {
  Serial.begin(9600);
  for (byte i = 0; i < 6; i++) {
    pinMode(BUTTON_PINS[i], INPUT);
    lastRaw[i] = digitalRead(BUTTON_PINS[i]);
    stableState[i] = lastRaw[i];
    lastChangeMs[i] = millis();
  }

  Serial.println(F("Six-button test: A0-A5"));
  Serial.println(F("Expected wiring: external pull-down, released=LOW, pressed=HIGH."));
  Serial.println(F("Press and release every button; only state changes are printed."));
}

void loop() {
  unsigned long now = millis();

  for (byte i = 0; i < 6; i++) {
    bool raw = digitalRead(BUTTON_PINS[i]);

    if (raw != lastRaw[i]) {
      lastRaw[i] = raw;
      lastChangeMs[i] = now;
    }

    if ((now - lastChangeMs[i]) >= DEBOUNCE_MS && raw != stableState[i]) {
      stableState[i] = raw;
      Serial.print(BUTTON_NAMES[i]);
      Serial.print(F(": "));
      Serial.println(stableState[i] == HIGH ? F("PRESSED (HIGH)") : F("RELEASED (LOW)"));
    }
  }
}
