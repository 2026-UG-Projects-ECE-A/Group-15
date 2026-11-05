/*
  Three-floor model lift controller using an ultrasonic distance sensor
  instead of floor limit switches.

  Floor distance calibration (distance from the fixed sensor to the lift target):
    Floor 1 =  2 cm
    Floor 2 = 20 cm
    Floor 3 = 40 cm

  Hardware/pin assumptions (based on the supplied smart_lift.ino):
    Height ultrasonic sensor: TRIG D3, ECHO D4
    Cabin occupancy ultrasonic sensor: TRIG D5, ECHO D6
    74HC595 / 7-segment display: DATA D7, LATCH D8, CLOCK D9
    Outside call buttons: A0, A1, A2
    Inside destination buttons: A3, A4, A5
    Motor driver: UP D12, DOWN D13

  Button inputs are configured as INPUT and are assumed to have external
  pull-down resistors (pressed = HIGH). Adjust if your wiring differs.

  This is for a model/prototype lift, not a passenger lift. Keep independent
  hardware end-stop protection in the real mechanism.
*/

#include <Arduino.h>

// -------------------- Display --------------------
const byte displayClockPin = 9;
const byte displayDataPin  = 7;
const byte displayLatchPin = 8;

// Common-anode 7-segment patterns for digits 0-3.
const byte DISPLAY_NUM[] = {
  0b11000000,  // 0: unknown / startup / fault
  0b11111001,  // 1
  0b10100100,  // 2
  0b10110000   // 3
};

// -------------------- Floor height sensor --------------------
const byte heightTrigPin = 3;
const byte heightEchoPin = 4;

// Explicit measured distance for each floor; do not derive from spacing.
const int FLOOR_1_DISTANCE_CM = 2;
const int FLOOR_2_DISTANCE_CM = 20;
const int FLOOR_3_DISTANCE_CM = 40;

// Allowed deviation around each calibrated floor distance.
const int FLOOR_TOLERANCE_CM = 2;

// Three-reading median filter; floor must be detected more than once.
const byte DISTANCE_HISTORY_SIZE = 3;
const byte FLOOR_CONFIRM_COUNT = 2;
const unsigned long FLOOR_SENSOR_INTERVAL_MS = 35;
const unsigned long FLOOR_ECHO_TIMEOUT_US = 25000UL;
const byte MAX_CONSECUTIVE_BAD_FLOOR_READS = 5;

// -------------------- Cabin occupancy sensor --------------------
const byte cabinTrigPin = 5;
const byte cabinEchoPin = 6;
const int CABIN_OCCUPIED_THRESHOLD_CM = 8;
const unsigned long CABIN_SENSOR_INTERVAL_MS = 250;
const unsigned long CABIN_ECHO_TIMEOUT_US = 12000UL;
const byte OCCUPIED_CONFIRM_COUNT = 2;
const byte EMPTY_CONFIRM_COUNT = 3;

// -------------------- Buttons --------------------
const byte outsideButtonPins[3] = {A0, A1, A2};
const byte insideButtonPins[3]  = {A3, A4, A5};
const unsigned long BUTTON_DEBOUNCE_MS = 35;

struct ButtonDebounce {
  byte pin;
  bool lastRawState;
  bool stableState;
  unsigned long lastChangeMs;
};

ButtonDebounce buttons[6];

// -------------------- Motor driver --------------------
const byte motorUpPin = 12;
const byte motorDownPin = 13;

// Motor outputs are assumed active-HIGH, matching the supplied sketch.

// -------------------- Lift behavior --------------------
const unsigned long DOOR_WAIT_MS = 3000;
const unsigned long STARTUP_CHECK_TIMEOUT_MS = 3000;
const unsigned long HOMING_TIMEOUT_MS = 25000;
const unsigned long TRAVEL_TIMEOUT_MS = 30000;

// -------------------- State and requests --------------------
enum ElevatorState {
  STARTUP_CHECK,
  HOMING_DOWN,
  IDLE,
  MOVING_UP,
  MOVING_DOWN,
  WAITING,
  FAULT
};

ElevatorState currentState = STARTUP_CHECK;

bool insideRequests[3]  = {false, false, false};
bool outsideRequests[3] = {false, false, false};

// currentFloor is the last reliably confirmed floor (1, 2, or 3); 0 = unknown.
int currentFloor = 0;
int lastDisplayedFloor = -1;
bool atFloor = false;
bool cabinOccupied = false;
int currentDirection = 0;  // 1 = up, -1 = down, 0 = undecided
int movementOriginFloor = 0;

unsigned long stateStartMs = 0;
unsigned long doorWaitStartMs = 0;
unsigned long lastFloorSensorMs = 0;
unsigned long lastCabinSensorMs = 0;
unsigned long startupStartMs = 0;
unsigned long lastDistancePrintMs = 0;

int distanceHistory[DISTANCE_HISTORY_SIZE] = {0};
byte distanceHistoryCount = 0;
byte distanceHistoryNext = 0;
int latestDistanceCm = -1;
int filteredDistanceCm = -1;
int candidateFloor = 0;
byte candidateFloorCount = 0;
byte consecutiveBadFloorReads = 0;
byte startupValidSamples = 0;
byte occupiedVotes = 0;
byte emptyVotes = 0;

// -------------------- Helper functions --------------------

void stopMotor() {
  digitalWrite(motorUpPin, LOW);
  digitalWrite(motorDownPin, LOW);
}

void applyMotorOutputs() {
  // Never energize the up and down outputs at the same time.
  if (currentState == HOMING_DOWN || currentState == MOVING_DOWN) {
    digitalWrite(motorUpPin, LOW);
    digitalWrite(motorDownPin, HIGH);
  } else if (currentState == MOVING_UP) {
    digitalWrite(motorUpPin, HIGH);
    digitalWrite(motorDownPin, LOW);
  } else {
    stopMotor();
  }
}

void enterFault(const __FlashStringHelper *reason) {
  stopMotor();
  currentState = FAULT;
  atFloor = false;
  Serial.print(F("FAULT: "));
  Serial.println(reason);
  Serial.println(F("Motor stopped. Reset the controller after inspecting the cause."));
}

void updateDisplay() {
  if (lastDisplayedFloor == currentFloor) return;

  int digit = currentFloor;
  if (digit < 0 || digit > 3) digit = 0;

  digitalWrite(displayLatchPin, LOW);
  shiftOut(displayDataPin, displayClockPin, MSBFIRST, DISPLAY_NUM[digit]);
  digitalWrite(displayLatchPin, HIGH);

  lastDisplayedFloor = currentFloor;
  Serial.print(F("Display updated to: "));
  Serial.println(digit);
}

int distanceForFloor(int floorNumber) {
  switch (floorNumber) {
    case 1: return FLOOR_1_DISTANCE_CM;
    case 2: return FLOOR_2_DISTANCE_CM;
    case 3: return FLOOR_3_DISTANCE_CM;
    default: return -1;
  }
}

int classifyFloor(int distanceCm) {
  if (distanceCm < 0) return 0;

  if (abs(distanceCm - FLOOR_1_DISTANCE_CM) <= FLOOR_TOLERANCE_CM) return 1;
  if (abs(distanceCm - FLOOR_2_DISTANCE_CM) <= FLOOR_TOLERANCE_CM) return 2;
  if (abs(distanceCm - FLOOR_3_DISTANCE_CM) <= FLOOR_TOLERANCE_CM) return 3;

  return 0;  // Between calibrated floor ranges.
}

int measureDistanceCm(byte trigPin, byte echoPin, unsigned long timeoutUs) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, timeoutUs);
  if (duration == 0) return -1;

  // Round instead of truncating, which matters for a 2 cm floor position.
  float distance = (duration * 0.0343f) / 2.0f;
  return (int)(distance + 0.5f);
}

int medianOfHistory() {
  int values[DISTANCE_HISTORY_SIZE];
  for (byte i = 0; i < DISTANCE_HISTORY_SIZE; i++) {
    values[i] = distanceHistory[i];
  }

  // Small insertion sort; history contains only three entries.
  for (byte i = 1; i < DISTANCE_HISTORY_SIZE; i++) {
    int value = values[i];
    int j = (int)i - 1;
    while (j >= 0 && values[j] > value) {
      values[j + 1] = values[j];
      j--;
    }
    values[j + 1] = value;
  }
  return values[DISTANCE_HISTORY_SIZE / 2];
}

void addDistanceToHistory(int distanceCm) {
  distanceHistory[distanceHistoryNext] = distanceCm;
  distanceHistoryNext = (distanceHistoryNext + 1) % DISTANCE_HISTORY_SIZE;
  if (distanceHistoryCount < DISTANCE_HISTORY_SIZE) distanceHistoryCount++;
}

void updateFloorSensor() {
  unsigned long now = millis();
  if (now - lastFloorSensorMs < FLOOR_SENSOR_INTERVAL_MS) return;
  lastFloorSensorMs = now;

  int measured = measureDistanceCm(heightTrigPin, heightEchoPin, FLOOR_ECHO_TIMEOUT_US);

  if (measured < 0) {
    if (consecutiveBadFloorReads < 255) consecutiveBadFloorReads++;

    if (currentState == STARTUP_CHECK) {
      // Startup timeout is handled by the state machine below.
    } else if ((currentState == HOMING_DOWN || currentState == MOVING_UP || currentState == MOVING_DOWN) &&
               consecutiveBadFloorReads >= MAX_CONSECUTIVE_BAD_FLOOR_READS) {
      enterFault(F("Repeated height-sensor timeouts while the motor was moving."));
    }
    return;
  }

  consecutiveBadFloorReads = 0;
  latestDistanceCm = measured;
  if (currentState == STARTUP_CHECK && startupValidSamples < 255) startupValidSamples++;

  addDistanceToHistory(measured);
  if (distanceHistoryCount < DISTANCE_HISTORY_SIZE) return;

  filteredDistanceCm = medianOfHistory();
  int detected = classifyFloor(filteredDistanceCm);

  if (detected == 0) {
    // The lift is between floor zones; keep the last confirmed floor number,
    // but do not treat the lift as currently stopped at that floor.
    atFloor = false;
    candidateFloor = 0;
    candidateFloorCount = 0;
  } else {
    if (detected == candidateFloor) {
      if (candidateFloorCount < 255) candidateFloorCount++;
    } else {
      candidateFloor = detected;
      candidateFloorCount = 1;
    }

    if (candidateFloorCount >= FLOOR_CONFIRM_COUNT) {
      atFloor = true;
      if (currentFloor != detected) {
        currentFloor = detected;
        Serial.print(F("Floor position confirmed: Floor "));
        Serial.print(currentFloor);
        Serial.print(F(" (filtered distance "));
        Serial.print(filteredDistanceCm);
        Serial.println(F(" cm)"));
      }
    }
  }

  if (now - lastDistancePrintMs >= 300) {
    lastDistancePrintMs = now;
    Serial.print(F("Height distance: "));
    Serial.print(latestDistanceCm);
    Serial.print(F(" cm; filtered: "));
    Serial.print(filteredDistanceCm);
    Serial.println(F(" cm"));
  }
}

void updateCabinOccupancy() {
  unsigned long now = millis();
  if (now - lastCabinSensorMs < CABIN_SENSOR_INTERVAL_MS) return;
  lastCabinSensorMs = now;

  int distanceCm = measureDistanceCm(cabinTrigPin, cabinEchoPin, CABIN_ECHO_TIMEOUT_US);
  if (distanceCm < 0) return;  // Keep the last stable occupancy state on a bad reading.

  if (distanceCm <= CABIN_OCCUPIED_THRESHOLD_CM) {
    emptyVotes = 0;
    if (occupiedVotes < 255) occupiedVotes++;
    if (occupiedVotes >= OCCUPIED_CONFIRM_COUNT && !cabinOccupied) {
      cabinOccupied = true;
      Serial.println(F("Cabin occupancy detected."));
    }
  } else {
    occupiedVotes = 0;
    if (emptyVotes < 255) emptyVotes++;
    if (emptyVotes >= EMPTY_CONFIRM_COUNT && cabinOccupied) {
      cabinOccupied = false;
      Serial.println(F("Cabin appears empty."));
    }
  }
}

void logRequest(const __FlashStringHelper *kind, int floorNumber) {
  Serial.print(kind);
  Serial.print(F(" request logged: Floor "));
  Serial.println(floorNumber);
}

void addOutsideRequest(byte floorIndex) {
  if (floorIndex >= 3) return;
  if (!outsideRequests[floorIndex]) {
    outsideRequests[floorIndex] = true;
    logRequest(F("Outside"), floorIndex + 1);
  }
}

void addInsideRequest(byte floorIndex) {
  if (floorIndex >= 3) return;
  if (!insideRequests[floorIndex]) {
    insideRequests[floorIndex] = true;
    logRequest(F("Inside"), floorIndex + 1);
  }
}

void updateButtons() {
  unsigned long now = millis();

  for (byte i = 0; i < 6; i++) {
    bool rawState = (digitalRead(buttons[i].pin) == HIGH);

    if (rawState != buttons[i].lastRawState) {
      buttons[i].lastRawState = rawState;
      buttons[i].lastChangeMs = now;
    }

    if ((now - buttons[i].lastChangeMs) >= BUTTON_DEBOUNCE_MS &&
        rawState != buttons[i].stableState) {
      buttons[i].stableState = rawState;

      // Register one request on the stable LOW -> HIGH transition.
      if (buttons[i].stableState) {
        if (i < 3) addOutsideRequest(i);
        else addInsideRequest(i - 3);
      }
    }
  }
}

bool hasRequestAtFloor(int floorNumber) {
  if (floorNumber < 1 || floorNumber > 3) return false;
  byte index = (byte)(floorNumber - 1);
  return outsideRequests[index] || (cabinOccupied && insideRequests[index]);
}

void clearRequestsAtFloor(int floorNumber) {
  if (floorNumber < 1 || floorNumber > 3) return;
  byte index = (byte)(floorNumber - 1);
  insideRequests[index] = false;
  outsideRequests[index] = false;
}

bool anyRequestAbove(int floorNumber) {
  for (int floor = floorNumber + 1; floor <= 3; floor++) {
    if (hasRequestAtFloor(floor)) return true;
  }
  return false;
}

bool anyRequestBelow(int floorNumber) {
  for (int floor = floorNumber - 1; floor >= 1; floor--) {
    if (hasRequestAtFloor(floor)) return true;
  }
  return false;
}

void beginWaitingAtCurrentFloor() {
  stopMotor();
  clearRequestsAtFloor(currentFloor);
  currentState = WAITING;
  doorWaitStartMs = millis();
  stateStartMs = doorWaitStartMs;
  Serial.print(F("Arrived at requested Floor "));
  Serial.println(currentFloor);
  Serial.println(F("Doors opening; waiting 3 seconds."));
}

void startHomingDown() {
  if (filteredDistanceCm >= 0 && filteredDistanceCm <= FLOOR_1_DISTANCE_CM + FLOOR_TOLERANCE_CM) {
    enterFault(F("Startup distance is near Floor 1 but the floor could not be confirmed."));
    return;
  }

  currentDirection = -1;
  movementOriginFloor = currentFloor;
  stateStartMs = millis();
  currentState = HOMING_DOWN;
  applyMotorOutputs();
  Serial.println(F("Homing: moving downward until Floor 1 is confirmed."));
}

void startMoving(int direction) {
  if (!atFloor || currentFloor < 1 || currentFloor > 3) return;

  currentDirection = direction;
  movementOriginFloor = currentFloor;
  stateStartMs = millis();
  currentState = (direction > 0) ? MOVING_UP : MOVING_DOWN;
  applyMotorOutputs();

  Serial.print(F("Moving "));
  Serial.println(direction > 0 ? F("UP") : F("DOWN"));
}

void processStateMachine() {
  unsigned long now = millis();

  switch (currentState) {
    case STARTUP_CHECK:
      stopMotor();

      if (atFloor && currentFloor == 1) {
        currentDirection = 0;
        currentState = IDLE;
        Serial.println(F("Startup check complete: already at Floor 1."));
      } else if (startupValidSamples >= 7) {
        if (filteredDistanceCm < 0) {
          enterFault(F("Could not obtain a usable startup distance."));
        } else {
          startHomingDown();
        }
      } else if (now - startupStartMs >= STARTUP_CHECK_TIMEOUT_MS) {
        enterFault(F("No reliable ultrasonic readings during startup."));
      }
      break;

    case HOMING_DOWN:
      if (atFloor && currentFloor == 1) {
        stopMotor();
        currentDirection = 0;
        currentState = IDLE;
        Serial.println(F("Homing complete at Floor 1."));
      } else if (now - stateStartMs >= HOMING_TIMEOUT_MS) {
        enterFault(F("Homing timed out before Floor 1 was confirmed."));
      }
      break;

    case IDLE: {
      stopMotor();

      // Preserve the original behavior: inside requests are discarded when
      // the occupancy sensor says the cabin is empty.
      if (!cabinOccupied) {
        for (byte i = 0; i < 3; i++) insideRequests[i] = false;
      }

      // Only schedule travel when the position sensor confirms a floor.
      if (!atFloor || currentFloor < 1 || currentFloor > 3) break;

      // Serve a call for the floor where the lift is already stopped.
      if (hasRequestAtFloor(currentFloor)) {
        beginWaitingAtCurrentFloor();
        break;
      }

      bool requestAbove = anyRequestAbove(currentFloor);
      bool requestBelow = anyRequestBelow(currentFloor);

      // SCAN-style direction choice: continue in the current direction if
      // requests remain that way; reverse only when none remain ahead.
      if (currentDirection >= 0) {
        if (requestAbove) startMoving(1);
        else if (requestBelow) startMoving(-1);
        else currentDirection = 0;
      } else {
        if (requestBelow) startMoving(-1);
        else if (requestAbove) startMoving(1);
        else currentDirection = 0;
      }
      break;
    }

    case MOVING_UP:
    case MOVING_DOWN:
      if (now - stateStartMs >= TRAVEL_TIMEOUT_MS) {
        enterFault(F("Travel timed out before a requested floor was reached."));
        break;
      }

      // Do not stop at the floor we departed from. Stop at any confirmed
      // requested floor reached in the direction of travel.
      if (atFloor && currentFloor != movementOriginFloor && hasRequestAtFloor(currentFloor)) {
        beginWaitingAtCurrentFloor();
        break;
      }

      // Do not keep driving into a mechanical end when a request appears
      // behind the lift after it has passed that request's floor. Reverse at
      // the terminal floor without opening the doors unless that floor was
      // itself requested.
      if (atFloor && currentFloor != movementOriginFloor &&
          currentState == MOVING_UP && currentFloor == 3) {
        stopMotor();
        if (anyRequestBelow(3)) {
          currentDirection = -1;
          Serial.println(F("Top floor reached; reversing to serve lower requests."));
        } else {
          currentDirection = 0;
          Serial.println(F("Top floor reached; no pending requests ahead."));
        }
        currentState = IDLE;
      } else if (atFloor && currentFloor != movementOriginFloor &&
                 currentState == MOVING_DOWN && currentFloor == 1) {
        stopMotor();
        if (anyRequestAbove(1)) {
          currentDirection = 1;
          Serial.println(F("Bottom floor reached; reversing to serve upper requests."));
        } else {
          currentDirection = 0;
          Serial.println(F("Bottom floor reached; no pending requests ahead."));
        }
        currentState = IDLE;
      }
      break;

    case WAITING:
      stopMotor();
      if (now - doorWaitStartMs >= DOOR_WAIT_MS) {
        Serial.println(F("Doors closed; resuming request scheduling."));
        currentState = IDLE;
      }
      break;

    case FAULT:
      stopMotor();
      break;
  }
}

// -------------------- Arduino setup/loop --------------------

void setup() {
  Serial.begin(9600);

  pinMode(displayLatchPin, OUTPUT);
  pinMode(displayClockPin, OUTPUT);
  pinMode(displayDataPin, OUTPUT);

  pinMode(heightTrigPin, OUTPUT);
  pinMode(heightEchoPin, INPUT);
  digitalWrite(heightTrigPin, LOW);

  pinMode(cabinTrigPin, OUTPUT);
  pinMode(cabinEchoPin, INPUT);
  digitalWrite(cabinTrigPin, LOW);

  for (byte i = 0; i < 3; i++) {
    pinMode(outsideButtonPins[i], INPUT);
    pinMode(insideButtonPins[i], INPUT);
    buttons[i].pin = outsideButtonPins[i];
    buttons[i + 3].pin = insideButtonPins[i];
  }

  for (byte i = 0; i < 6; i++) {
    bool initialState = (digitalRead(buttons[i].pin) == HIGH);
    buttons[i].lastRawState = initialState;
    buttons[i].stableState = initialState;
    buttons[i].lastChangeMs = millis();
  }

  pinMode(motorUpPin, OUTPUT);
  pinMode(motorDownPin, OUTPUT);
  stopMotor();

  currentFloor = 0;
  currentState = STARTUP_CHECK;
  startupStartMs = millis();
  lastFloorSensorMs = millis() - FLOOR_SENSOR_INTERVAL_MS;
  lastCabinSensorMs = millis();

  updateDisplay();
  Serial.println(F("Three-floor ultrasonic lift controller starting."));
  Serial.println(F("Calibrated floor distances: F1=2 cm, F2=20 cm, F3=40 cm."));
  Serial.println(F("Keep clear of the mechanism during startup homing."));
}

void loop() {
  updateFloorSensor();
  updateCabinOccupancy();
  updateButtons();
  processStateMachine();
  applyMotorOutputs();
  updateDisplay();
}
