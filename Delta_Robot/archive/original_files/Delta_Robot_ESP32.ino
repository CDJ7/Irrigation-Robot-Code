/*
  =======================================================================
  Delta Robot Slave Controller — ESP32 + 3x DM542T + NEMA23
  =======================================================================
  ROLE: This ESP32 does ZERO kinematics. It only:
    - Receives absolute step targets for 3 axes + a move duration (ms)
    - Pulses STEP/DIR pins so all 3 axes arrive at their targets at the
      same time (synchronized multi-axis move, classic DDA/Bresenham)
    - Runs a homing routine using 3 limit switches
    - Enables/disables the drivers and reports current step position

  It talks to the Python "master" GUI over USB serial at 115200 baud,
  using this line-based text protocol:

    M <s1> <s2> <s3> <duration_ms>   -> move all 3 axes to absolute step
                                         counts s1,s2,s3, arriving together
                                         after duration_ms milliseconds
    H                                -> run homing routine on all 3 axes
    Z                                -> Zero coordinates (force homed state)
    EN 1                             -> enable all drivers
    EN 0                             -> disable all drivers
    STOP                             -> abort any move immediately
    P?                               -> reply "POS s1 s2 s3"

  Responses sent back up to Python (all newline terminated):
    POS s1 s2 s3     - current absolute step position
    HOMED            - homing routine finished, position reset to 0 0 0
    OK               - move finished normally
    ERR <msg>        - something went wrong (bad command, homing timeout..)

  =======================================================================
  WIRING (edit PIN definitions below to match your actual wiring)
  =======================================================================
  Each DM542T needs: PUL+ (step), DIR+ (direction), ENA+ (enable).
  Tie all three drivers' PUL-, DIR-, ENA- to a common ESP32 GND (or to
  3.3V and invert logic in software — see STEP_ACTIVE_LOW etc. below if
  your driver's opto inputs are wired common-anode instead of common-cathode).

  NOTE ON LOGIC LEVELS: DM542T opto-inputs are commonly used at 5V but
  will usually also trigger fine at 3.3V (ESP32 logic level) as long as
  you're within the driver's minimum trigger current — check your DM542T
  manual's opto-input current spec before relying on this. If steps get
  missed/skipped, this is the first thing to check (undersized/oversized
  series resistor on the opto input, or level not reaching the threshold).

  Pins chosen below avoid ESP32 boot-strapping pins (0, 2, 12, 15) and
  flash pins (6-11), and avoid input-only pins (34-39) so limit switches
  can use internal pull-ups.

  Axis 0 (A): STEP=25  DIR=32  EN=5   LIMIT=21
  Axis 1 (B): STEP=26  DIR=33  EN=18  LIMIT=22
  Axis 2 (C): STEP=27  DIR=4   EN=19  LIMIT=23

  Limit switches: wired NO (normally open) between the GPIO and GND,
  using the ESP32's internal pull-up (INPUT_PULLUP). Triggered = LOW.
*/

#include <Arduino.h>

// =========================================================
//                      CONFIGURATION
// =========================================================

const uint8_t NUM_AXES = 3;

const uint8_t STEP_PIN[NUM_AXES]  = {25, 26, 27};
const uint8_t DIR_PIN[NUM_AXES]   = {32, 33, 4};
const uint8_t EN_PIN[NUM_AXES]    = {5,  18, 19};
const uint8_t LIMIT_PIN[NUM_AXES] = {21, 22, 23};

// DM542T ENA input: LOW = driver enabled, HIGH = driver disabled
// (this is the typical/default DM542T behavior — flip if yours differs)
const bool EN_ACTIVE_LOW = true;

// Minimum STEP pulse width. DM542T datasheet typically wants >=2.5us high
// and >=2.5us low; 5us gives margin.
const uint16_t STEP_PULSE_US = 5;

// DIR must be stable for some minimum time before/after a STEP edge.
// DM542T wants >=5us setup time; we use 10us for margin.
const uint16_t DIR_SETUP_US = 10;

// Homing parameters — TUNE THESE for your machine before running.
// Homing direction: -1 or +1, whichever way drives the arm toward the
// limit switch. Delta robots usually home "up" (effector rises to top).
// Changed to +1 to stop crashing the other direction.
const int8_t HOME_DIR[NUM_AXES] = {1, 1, 1}; 

// Step delay while homing (controls homing speed). Larger = slower/safer.
const uint16_t HOMING_STEP_DELAY_US = 900;

// After a limit switch triggers, back off this many steps before calling
// that axis "home" and zeroing it, so it isn't sitting exactly on the switch.
const long BACKOFF_STEPS = 200;

// Safety cap so a broken/unwired limit switch can't home forever.
const long HOMING_MAX_STEPS = 200000;

const long SERIAL_BAUD = 115200;

// =========================================================
//                      RUNTIME STATE
// =========================================================

long currentSteps[NUM_AXES] = {0, 0, 0};

bool moving = false;
long targetSteps[NUM_AXES];
long absSteps[NUM_AXES];
int8_t dirSign[NUM_AXES];
long error_[NUM_AXES];
uint8_t masterAxis = 0;
long masterStepsTotal = 0;
long ticksDone = 0;
uint64_t nextTickTime_us = 0;
uint64_t tickInterval_us = 0;

String serialBuf = "";

// =========================================================
//                      LOW-LEVEL HELPERS
// =========================================================

void setDriverEnable(bool enable) {
  bool pinLevel = EN_ACTIVE_LOW ? !enable : enable;
  for (uint8_t i = 0; i < NUM_AXES; i++) {
    digitalWrite(EN_PIN[i], pinLevel ? HIGH : LOW);
  }
}

// Pulses one axis's STEP pin. Assumes DIR is already set correctly.
inline void pulseStep(uint8_t axis) {
  digitalWrite(STEP_PIN[axis], HIGH);
  delayMicroseconds(STEP_PULSE_US);
  digitalWrite(STEP_PIN[axis], LOW);
  delayMicroseconds(STEP_PULSE_US);
}

void setDir(uint8_t axis, int8_t sign) {
  digitalWrite(DIR_PIN[axis], sign >= 0 ? HIGH : LOW);
}

void sendPos() {
  Serial.print("POS ");
  Serial.print(currentSteps[0]);
  Serial.print(' ');
  Serial.print(currentSteps[1]);
  Serial.print(' ');
  Serial.println(currentSteps[2]);
}

// =========================================================
//                      MOVE (SYNCHRONIZED DDA)
// =========================================================
// All axes are driven from a single "master" axis (the one needing the
// most steps). Every time the master ticks, the other axes accumulate
// error and step whenever their accumulated error exceeds the master's
// step count -- this is the same trick used in CNC firmware (Bresenham
// line algorithm extended to N axes) and guarantees every axis finishes
// its steps at the same tick the master finishes.

void startMove(long t0, long t1, long t2, unsigned long durationMs) {
  targetSteps[0] = t0;
  targetSteps[1] = t1;
  targetSteps[2] = t2;

  long delta[NUM_AXES];
  masterStepsTotal = 0;
  masterAxis = 0;

  for (uint8_t i = 0; i < NUM_AXES; i++) {
    delta[i] = targetSteps[i] - currentSteps[i];
    absSteps[i] = labs(delta[i]);
    dirSign[i] = (delta[i] >= 0) ? 1 : -1;
    error_[i] = 0;
    if (absSteps[i] > masterStepsTotal) {
      masterStepsTotal = absSteps[i];
      masterAxis = i;
    }
  }

  if (masterStepsTotal == 0) {
    // Nothing to move — already at target.
    Serial.println("OK");
    return;
  }

  // Set all DIR pins up front, then respect setup time once.
  for (uint8_t i = 0; i < NUM_AXES; i++) {
    setDir(i, dirSign[i]);
  }
  delayMicroseconds(DIR_SETUP_US);

  if (durationMs < 1) durationMs = 1;
  tickInterval_us = ((uint64_t)durationMs * 1000ULL) / (uint64_t)masterStepsTotal;
  if (tickInterval_us < (STEP_PULSE_US * 2 + 2)) {
    // Requested duration is faster than the driver/pulse width can do.
    // Clamp so we don't generate an invalid/overlapping pulse train.
    tickInterval_us = STEP_PULSE_US * 2 + 2;
  }

  ticksDone = 0;
  nextTickTime_us = micros();
  moving = true;
}

// Call every loop() iteration. Non-blocking except for the brief
// STEP_PULSE_US pulse widths when a tick actually fires.
void serviceMove() {
  if (!moving) return;

  if ((int64_t)(micros() - nextTickTime_us) < 0) {
    return; // not time for the next tick yet
  }

  // Master axis always steps this tick.
  pulseStep(masterAxis);
  currentSteps[masterAxis] += dirSign[masterAxis];

  // Other axes step only if their accumulated error says so.
  for (uint8_t i = 0; i < NUM_AXES; i++) {
    if (i == masterAxis) continue;
    error_[i] += absSteps[i];
    if (error_[i] >= masterStepsTotal) {
      error_[i] -= masterStepsTotal;
      pulseStep(i);
      currentSteps[i] += dirSign[i];
    }
  }

  ticksDone++;
  nextTickTime_us += tickInterval_us;

  if (ticksDone >= masterStepsTotal) {
    moving = false;
    Serial.println("OK");
  }
}

void abortMove() {
  if (moving) {
    moving = false;
    Serial.println("ERR STOPPED");
  }
  sendPos();
}

// =========================================================
//                      HOMING
// =========================================================
// Moves all not-yet-triggered axes together (slowly) toward their limit
// switches. Each axis stops as soon as its own switch triggers. Once all
// three are triggered, backs every axis off a fixed number of steps and
// zeros the position. Blocking, but checks serial for STOP so you're not
// stuck forever if a switch is unwired/broken.

bool checkForStopDuringHoming() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line == "STOP") return true;
  }
  return false;
}

void runHoming() {
  bool doneAxis[NUM_AXES] = {false, false, false};
  long stepsTaken[NUM_AXES] = {0, 0, 0};

  for (uint8_t i = 0; i < NUM_AXES; i++) {
    setDir(i, HOME_DIR[i]);
  }
  delayMicroseconds(DIR_SETUP_US);

  bool allDone = false;
  while (!allDone) {
    allDone = true;
    for (uint8_t i = 0; i < NUM_AXES; i++) {
      if (doneAxis[i]) continue;
      allDone = false;

      if (digitalRead(LIMIT_PIN[i]) == LOW) {
        doneAxis[i] = true;
        continue;
      }

      pulseStep(i);
      stepsTaken[i]++;

      if (stepsTaken[i] > HOMING_MAX_STEPS) {
        Serial.print("ERR HOMING_TIMEOUT_AXIS_");
        Serial.println(i);
        return;
      }
    }
    delayMicroseconds(HOMING_STEP_DELAY_US);

    if (checkForStopDuringHoming()) {
      Serial.println("ERR HOMING_ABORTED");
      return;
    }
  }

  // Back off every axis away from its switch.
  for (uint8_t i = 0; i < NUM_AXES; i++) {
    setDir(i, -HOME_DIR[i]);
  }
  delayMicroseconds(DIR_SETUP_US);
  for (long s = 0; s < BACKOFF_STEPS; s++) {
    for (uint8_t i = 0; i < NUM_AXES; i++) {
      pulseStep(i);
    }
    delayMicroseconds(HOMING_STEP_DELAY_US);
  }

  for (uint8_t i = 0; i < NUM_AXES; i++) {
    currentSteps[i] = 0;
  }

  Serial.println("HOMED");
  sendPos();
}

// =========================================================
//                      SERIAL COMMAND PARSING
// =========================================================

void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line.startsWith("M ")) {
    long s0, s1, s2;
    unsigned long dur;
    int n = sscanf(line.c_str(), "M %ld %ld %ld %lu", &s0, &s1, &s2, &dur);
    if (n != 4) {
      Serial.println("ERR BAD_M_ARGS");
      return;
    }
    if (moving) {
      Serial.println("ERR BUSY");
      return;
    }
    startMove(s0, s1, s2, dur);

  } else if (line == "H") {
    if (moving) {
      Serial.println("ERR BUSY");
      return;
    }
    runHoming();

  } else if (line == "Z") {
    // Force reset of current coordinates to 0 without moving
    for (uint8_t i = 0; i < NUM_AXES; i++) {
      currentSteps[i] = 0;
    }
    Serial.println("HOMED");
    
  } else if (line.startsWith("EN")) {
    if (line.indexOf('1') >= 0) {
      setDriverEnable(true);
      Serial.println("OK");
    } else {
      setDriverEnable(false);
      Serial.println("OK");
    }

  } else if (line == "STOP") {
    abortMove();

  } else if (line == "P?") {
    sendPos();

  } else {
    Serial.print("ERR UNKNOWN_CMD ");
    Serial.println(line);
  }
}

// =========================================================
//                      ARDUINO ENTRY POINTS
// =========================================================

void setup() {
  Serial.begin(SERIAL_BAUD);

  for (uint8_t i = 0; i < NUM_AXES; i++) {
    pinMode(STEP_PIN[i], OUTPUT);
    pinMode(DIR_PIN[i], OUTPUT);
    pinMode(EN_PIN[i], OUTPUT);
    pinMode(LIMIT_PIN[i], INPUT_PULLUP);
    digitalWrite(STEP_PIN[i], LOW);
  }

  setDriverEnable(false); // start disabled for safety until GUI says EN 1

  Serial.println("READY DeltaRobotSlave");
}

void loop() {
  // Non-blocking move stepping.
  serviceMove();

  // Non-blocking-ish serial line read (one command per loop pass once
  // a full line has arrived).
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      handleCommand(serialBuf);
      serialBuf = "";
    } else if (c != '\r') {
      serialBuf += c;
    }
  }
}
