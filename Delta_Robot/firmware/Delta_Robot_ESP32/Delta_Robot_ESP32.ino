/*
  =======================================================================
  Delta Robot Slave Controller — ESP32 + 3x DM542T + NEMA23   (version 2)
  =======================================================================
  ROLE: This ESP32 does ZERO kinematics. It only:
    - Receives absolute step targets for 3 axes + a move duration (ms)
    - Pulses STEP/DIR pins so all 3 axes arrive at their targets at the
      same time (synchronized multi-axis move, classic DDA/Bresenham),
      now with a trapezoidal speed profile (gentle start and stop)
    - Runs a homing routine using 3 (debounced) limit switches
    - Switches the vacuum on and off (GPIO 17)
    - Enables/disables the drivers and reports current step position

  It talks to the Python "master" GUI over USB serial at 115200 baud,
  using this line-based text protocol (fully backward compatible with
  version 1: every old command and reply is unchanged):

    M <s1> <s2> <s3> <duration_ms>   -> move all 3 axes to absolute step
                                         counts s1,s2,s3. duration_ms is the
                                         time at CRUISE speed; the speed ramps
                                         up and down, so the real time is a
                                         little longer (see ACCEL_STEPS_S2).
    H                                -> run homing routine on all 3 axes
    Z                                -> Zero coordinates (force homed state)
    EN 1 / EN 0                      -> enable / disable all drivers
    STOP                             -> abort any move (or homing) at once,
                                         AND switch the vacuum off
    P?                               -> reply "POS s1 s2 s3"
    V 1 / V 0        (NEW)           -> vacuum on / off, reply "OK"
    V?               (NEW)           -> reply "VAC 1" or "VAC 0"

  Responses sent back up to Python (all newline terminated):
    POS s1 s2 s3     - current absolute step position
    HOMED            - homing routine finished, position reset to 0 0 0
    OK               - move finished / EN or V command accepted
    ERR <msg>        - something went wrong (bad command, homing timeout..)
    WARN <msg>       - information only (e.g. WARN VACUUM_TIMEOUT)

  What changed from version 1 (see docs/FIRMWARE.md):
    * Trapezoidal acceleration / deceleration on every move.
    * Vacuum command V 1 / V 0 on GPIO 17 (active HIGH). OFF at boot, OFF on
      STOP, and auto-OFF after VACUUM_MAX_ON_MS (safety).
    * Homing: serial is read without blocking, commands other than STOP that
      arrive during homing are QUEUED (not thrown away) and run afterwards;
      limit switches are debounced; homing is refused when the drivers are
      disabled (ERR NOT_ENABLED).
    * EN and Z are refused while a move is running (ERR BUSY).
    * The move timer no longer breaks when micros() wraps (every 71 minutes).
    * Over-long garbage lines are discarded (ERR LINE_TOO_LONG).

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
  Vacuum:     GPIO 17 -> relay / MOSFET module input, ACTIVE HIGH (HIGH = on)

  VACUUM WIRING NOTES: GPIO 17 floats for a moment while the ESP32 boots.
  Put a 10 kOhm resistor from GPIO 17 to GND so the vacuum cannot twitch on
  during start-up or while the board is being flashed. The GPIO is 3.3 V and
  can supply only a few mA: use a module that is happy with 3.3 V on its
  input, never drive a pump or a relay coil directly from the pin. Share GND
  between the ESP32 and the module.

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

// Vacuum output. GPIO 17 is a normal output-capable pin (34-39 are input only).
const uint8_t VACUUM_PIN = 17;
const bool VACUUM_ACTIVE_HIGH = true;          // true: HIGH = vacuum on
// Safety: the vacuum switches itself off after this long (0 = never). The GUI never
// keeps it on for more than a few seconds; this protects against a crashed program.
const unsigned long VACUUM_MAX_ON_MS = 30000UL;

// DM542T ENA input: LOW = driver enabled, HIGH = driver disabled
// (this is the typical/default DM542T behavior — flip if yours differs)
const bool EN_ACTIVE_LOW = true;

// Minimum STEP pulse width. DM542T datasheet typically wants >=2.5us high
// and >=2.5us low; 5us gives margin.
const uint16_t STEP_PULSE_US = 5;

// DIR must be stable for some minimum time before/after a STEP edge.
// DM542T wants >=5us setup time; we use 10us for margin.
const uint16_t DIR_SETUP_US = 10;

// ---- Acceleration (trapezoidal profile) -- UNVERIFIED STARTING VALUES, tune on the machine ----
// Speeds are in STEPS PER SECOND of the fastest (master) axis. With 3200 pulses/rev and a 47:1
// gearbox, 417.8 steps = 1 degree of arm movement.
//   ACCEL_STEPS_S2 : how quickly the speed may change. Lower = gentler. 0 = no acceleration (old behaviour).
//   START_RATE     : speed at the very start and end of a move (must be a speed the motor can start at).
//   MAX_STEP_RATE  : hard ceiling, whatever the GUI asks for.
const float ACCEL_STEPS_S2 = 20000.0f;
const float START_RATE = 300.0f;
const float MAX_STEP_RATE = 30000.0f;

// Homing parameters — TUNE THESE for your machine before running.
// Homing direction: -1 or +1, whichever way drives the arm toward the
// limit switch. Delta robots usually home "up" (effector rises to top).
// Changed to +1 to stop crashing the other direction.
const int8_t HOME_DIR[NUM_AXES] = {1, 1, 1};

// Step delay while homing (controls homing speed). Larger = slower/safer.
const uint16_t HOMING_STEP_DELAY_US = 900;

// A limit switch counts as pressed only after this many CONSECUTIVE low readings (one reading per
// homing loop, about 1 ms apart). Filters electrical noise and contact bounce. 1 = no debounce.
const uint8_t LIMIT_DEBOUNCE_SAMPLES = 3;

// After a limit switch triggers, back off this many steps before calling
// that axis "home" and zeroing it, so it isn't sitting exactly on the switch.
// NOTE: 200 steps is only about 0.5 degree of arm movement with the 47:1 gearbox. Check by eye
// that the switch has released after homing; raise this if it has not.
const long BACKOFF_STEPS = 200;

// Safety cap so a broken/unwired limit switch can't home forever.
const long HOMING_MAX_STEPS = 200000;

const long SERIAL_BAUD = 115200;

const uint8_t MAX_LINE = 80;          // longer lines are garbage and are discarded
const uint8_t PENDING_MAX = 4;        // commands remembered while homing

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
// 32-bit microsecond timer arithmetic: (int32_t)(micros() - t) stays correct across the 71-minute wrap.
uint32_t nextTick_us = 0;
uint32_t tickInterval_us = 0;
float velocity = 0.0f;                // current speed, steps/s (master axis)
float cruiseRate = 0.0f;              // requested top speed, steps/s

bool driversEnabled = false;
bool vacuumOn = false;
unsigned long vacuumSince_ms = 0;

String serialBuf = "";
String pending[PENDING_MAX];
uint8_t pendingCount = 0;

// =========================================================
//                      LOW-LEVEL HELPERS
// =========================================================

void setDriverEnable(bool enable) {
  driversEnabled = enable;
  bool pinLevel = EN_ACTIVE_LOW ? !enable : enable;
  for (uint8_t i = 0; i < NUM_AXES; i++) {
    digitalWrite(EN_PIN[i], pinLevel ? HIGH : LOW);
  }
}

void setVacuum(bool on) {
  digitalWrite(VACUUM_PIN, (on == VACUUM_ACTIVE_HIGH) ? HIGH : LOW);
  if (on && !vacuumOn) vacuumSince_ms = millis();
  vacuumOn = on;
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

// Reads whatever has arrived, WITHOUT blocking. Returns true when one complete line is ready in `out`.
bool readLine(String &out) {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      out = serialBuf;
      serialBuf = "";
      return true;
    }
    if (c == '\r') continue;
    if (serialBuf.length() < MAX_LINE) {
      serialBuf += c;
    } else {
      serialBuf = "";
      Serial.println("ERR LINE_TOO_LONG");
    }
  }
  return false;
}

// =========================================================
//                      MOVE (SYNCHRONIZED DDA + TRAPEZOID)
// =========================================================
// All axes are driven from a single "master" axis (the one needing the
// most steps). Every time the master ticks, the other axes accumulate
// error and step whenever their accumulated error exceeds the master's
// step count -- this is the same trick used in CNC firmware (Bresenham
// line algorithm extended to N axes) and guarantees every axis finishes
// its steps at the same tick the master finishes.
//
// Speed profile: the master starts at START_RATE, accelerates at ACCEL_STEPS_S2 up to the cruise rate
// that the GUI's duration implies, and slows down in time to finish at START_RATE. Short moves never
// reach cruise speed (triangle profile). All other axes scale with the master, so they stay in sync.

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
  cruiseRate = (float)masterStepsTotal * 1000.0f / (float)durationMs;
  if (cruiseRate > MAX_STEP_RATE) cruiseRate = MAX_STEP_RATE;   // faster than the driver/pulse width can do
  if (cruiseRate < 1.0f) cruiseRate = 1.0f;

  velocity = (ACCEL_STEPS_S2 > 0.0f) ? fminf(START_RATE, cruiseRate) : cruiseRate;
  tickInterval_us = (uint32_t)(1000000.0f / velocity);

  ticksDone = 0;
  nextTick_us = micros();
  moving = true;
}

// Call every loop() iteration. Non-blocking except for the brief
// STEP_PULSE_US pulse widths when a tick actually fires.
void serviceMove() {
  if (!moving) return;

  if ((int32_t)(micros() - nextTick_us) < 0) {
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

  if (ticksDone >= masterStepsTotal) {
    moving = false;
    Serial.println("OK");
    return;
  }

  if (ACCEL_STEPS_S2 > 0.0f) {
    long remaining = masterStepsTotal - ticksDone;
    float floorRate = fminf(START_RATE, cruiseRate);
    float vBrake = sqrtf(2.0f * ACCEL_STEPS_S2 * (float)remaining);   // fastest speed from which we can still stop in time
    float vTarget = fminf(cruiseRate, fmaxf(vBrake, floorRate));
    float dv = ACCEL_STEPS_S2 / velocity;                             // a * dt, with dt = 1 / velocity
    if (velocity < vTarget) velocity = fminf(vTarget, velocity + dv);
    else if (velocity > vTarget) velocity = fmaxf(vTarget, velocity - dv);
  }
  tickInterval_us = (uint32_t)(1000000.0f / velocity);
  nextTick_us += tickInterval_us;

  // If the loop fell far behind (long serial print etc.), do not fire a burst of catch-up steps.
  if ((int32_t)(micros() - nextTick_us) > 2000) nextTick_us = micros();
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
// switches. Each axis stops as soon as its own switch triggers (debounced).
// Once all three are triggered, backs every axis off a fixed number of steps
// and zeros the position. Blocking, but serial is polled without blocking: STOP
// aborts homing, every other command is queued and executed after homing ends.

// Returns true if STOP was received.
bool serviceSerialDuringHoming() {
  String line;
  if (!readLine(line)) return false;
  line.trim();
  if (line.length() == 0) return false;
  if (line == "STOP") return true;
  if (pendingCount < PENDING_MAX) {
    pending[pendingCount++] = line;
  } else {
    Serial.println("ERR BUSY");
  }
  return false;
}

void runHoming() {
  bool doneAxis[NUM_AXES] = {false, false, false};
  uint8_t lowCount[NUM_AXES] = {0, 0, 0};
  long stepsTaken[NUM_AXES] = {0, 0, 0};

  // An axis that is already on its switch must not be driven further into it:
  // sample each switch a few times before moving anything.
  for (uint8_t s = 0; s < LIMIT_DEBOUNCE_SAMPLES; s++) {
    for (uint8_t i = 0; i < NUM_AXES; i++) {
      if (digitalRead(LIMIT_PIN[i]) == LOW) lowCount[i]++; else lowCount[i] = 0;
    }
    delayMicroseconds(HOMING_STEP_DELAY_US);
  }
  for (uint8_t i = 0; i < NUM_AXES; i++) {
    if (lowCount[i] >= LIMIT_DEBOUNCE_SAMPLES) doneAxis[i] = true;
  }

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
        if (++lowCount[i] >= LIMIT_DEBOUNCE_SAMPLES) {
          doneAxis[i] = true;
          continue;
        }
      } else {
        lowCount[i] = 0;
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

    if (serviceSerialDuringHoming()) {
      setVacuum(false);
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
    if ((s & 15) == 0 && serviceSerialDuringHoming()) {
      setVacuum(false);
      Serial.println("ERR HOMING_ABORTED");
      return;
    }
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
    if (!driversEnabled) {
      Serial.println("ERR NOT_ENABLED");      // homing with the drivers off would just count steps that never happen
      return;
    }
    runHoming();

  } else if (line == "Z") {
    if (moving) {
      Serial.println("ERR BUSY");
      return;
    }
    // Force reset of current coordinates to 0 without moving
    for (uint8_t i = 0; i < NUM_AXES; i++) {
      currentSteps[i] = 0;
    }
    Serial.println("HOMED");

  } else if (line.startsWith("EN")) {
    if (moving) {
      Serial.println("ERR BUSY");              // use STOP first
      return;
    }
    setDriverEnable(line.indexOf('1') >= 0);
    Serial.println("OK");

  } else if (line == "V 1") {
    if (moving) {
      Serial.println("ERR BUSY");
      return;
    }
    setVacuum(true);
    Serial.println("OK");

  } else if (line == "V 0") {
    setVacuum(false);                           // always allowed: switching OFF is the safe direction
    Serial.println("OK");

  } else if (line == "V?") {
    Serial.println(vacuumOn ? "VAC 1" : "VAC 0");

  } else if (line == "STOP") {
    setVacuum(false);
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
  // Vacuum first, and OFF, before anything else can go wrong.
  digitalWrite(VACUUM_PIN, VACUUM_ACTIVE_HIGH ? LOW : HIGH);
  pinMode(VACUUM_PIN, OUTPUT);
  digitalWrite(VACUUM_PIN, VACUUM_ACTIVE_HIGH ? LOW : HIGH);
  vacuumOn = false;

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

  // Vacuum safety timeout.
  if (vacuumOn && VACUUM_MAX_ON_MS > 0 && (millis() - vacuumSince_ms) > VACUUM_MAX_ON_MS) {
    setVacuum(false);
    Serial.println("WARN VACUUM_TIMEOUT");
  }

  // Commands that arrived while homing ran.
  if (pendingCount > 0 && !moving) {
    String cmd = pending[0];
    for (uint8_t i = 1; i < pendingCount; i++) pending[i - 1] = pending[i];
    pendingCount--;
    handleCommand(cmd);
    return;
  }

  // One serial line per pass keeps the step timing steady.
  String line;
  if (readLine(line)) {
    handleCommand(line);
  }
}
