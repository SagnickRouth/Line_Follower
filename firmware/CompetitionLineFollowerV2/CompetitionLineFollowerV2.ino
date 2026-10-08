/*
  Devta Robotics - Competition Line Follower V2

  ESP32-WROOM / ESP32 DevKit V1
  Driver : TB6612FNG
  Sensors: 16-channel analog IR array through a 4-bit analog MUX
  UI     : SSD1306 OLED + 6 buttons

  V2 goals:
    - Keep V1 untouched.
    - Do NOT claim a 1 kHz loop unless the measured frame timing supports it.
    - Target a deterministic 500 Hz control period by default.
    - Measure sensor scan, control execution, frame period and overruns.
    - Use real dt for I and D terms.
    - Filter the derivative term.
    - Use conditional anti-windup.
    - Validate calibration before running.
    - Improve line confidence and line-loss handling.
    - Keep the start ramp under PID control instead of driving blind.
    - Avoid flash writes unless settings changed.

  Important:
    The actual control frequency is limited by the complete 16-channel
    MUX + ADC frame time. The firmware reports the measured frequency.
    Tune CONTROL_PERIOD_US only after measuring SCAN/FRAME timing on hardware.
*/

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// -----------------------------------------------------------------------------
// Pin map
// -----------------------------------------------------------------------------

static const uint8_t SENSOR_COUNT = 16;

static const uint8_t PIN_SENSOR_MUX_OUT = 36;
static const uint8_t PIN_SENSOR_S0 = 32;
static const uint8_t PIN_SENSOR_S1 = 33;
static const uint8_t PIN_SENSOR_S2 = 25;
static const uint8_t PIN_SENSOR_S3 = 26;

static const uint8_t PIN_MOTOR_L_PWM = 23;
static const uint8_t PIN_MOTOR_L_IN1 = 18;
static const uint8_t PIN_MOTOR_L_IN2 = 19;
static const uint8_t PIN_MOTOR_R_PWM = 5;
static const uint8_t PIN_MOTOR_R_IN1 = 16;
static const uint8_t PIN_MOTOR_R_IN2 = 17;
static const uint8_t PIN_MOTOR_STBY = 4;

static const uint8_t PIN_BTN_NEXT = 13;
static const uint8_t PIN_BTN_PREV = 14;
static const uint8_t PIN_BTN_ENTER = 27;
static const uint8_t PIN_BTN_BACK = 12;
static const uint8_t PIN_BTN_CAL = 35;
static const uint8_t PIN_BTN_START = 34;

static const uint8_t PIN_I2C_SDA = 21;
static const uint8_t PIN_I2C_SCL = 22;

// -----------------------------------------------------------------------------
// Timing / control configuration
// -----------------------------------------------------------------------------

// 500 Hz target is deliberate. The complete 16-channel scan must fit inside
// this period with margin. The measured frame rate is shown in diagnostics.
static const uint32_t CONTROL_PERIOD_US = 2000;

static const uint32_t UI_PERIOD_MS = 100;
static const uint32_t TELEMETRY_PERIOD_MS = 500;
static const uint32_t BUTTON_PERIOD_MS = 10;

static const uint32_t CALIBRATION_TIME_MS = 5000;
static const uint32_t LINE_GAP_HOLD_MS = 100;
static const uint32_t LOST_LINE_SEARCH_MS = 900;

// Dead-end detection / U-turn maneuver.
// A dead end is confirmed only when the line was recently strong and centered,
// no side branch was recently seen, and the line remains absent long enough.
// This avoids treating ordinary sharp corners as dead ends.
static const uint32_t DEAD_END_CONFIRM_MS = 220;
static const uint32_t DEAD_END_MEMORY_MS = 180;
static const uint16_t DEAD_END_CENTER_TOLERANCE = 1600;
static const uint16_t DEAD_END_MIN_STRENGTH = 1400;
static const uint16_t DEAD_END_SIDE_THRESHOLD = 550;
static const uint8_t DEAD_END_SIDE_SENSORS = 3;

static const uint32_t UTURN_MIN_MS = 280;
static const uint32_t UTURN_REACQUIRE_MS = 45;
static const uint32_t UTURN_MAX_MS = 1500;
static const int16_t UTURN_POWER = 520;

// Maze solver configuration. The maze mode is a separate navigation layer
// above the same sensor/PID/motor drivers. It is intended for taped line mazes.
// The exploration pass uses a left-hand rule and records L/S/R/B moves. The
// recorded path is reduced with LSRB rules whenever a dead-end B is added.
static const uint32_t MAZE_JUNCTION_DEBOUNCE_MS = 180;
static const uint32_t MAZE_PROBE_MS = 90;
static const uint32_t MAZE_TURN_CLEAR_MS = 90;
static const uint32_t MAZE_LINE_REACQUIRE_MS = 35;
static const uint32_t MAZE_TURN_TIMEOUT_MS = 900;
static const uint32_t MAZE_DEAD_END_CONFIRM_MS = 90;
static const uint32_t MAZE_GOAL_CONFIRM_MS = 220;
static const uint32_t MAZE_START_IGNORE_MS = 600;

static const uint16_t MAZE_BRANCH_THRESHOLD = 500;
static const uint8_t MAZE_BRANCH_SENSORS = 4;
static const uint16_t MAZE_CENTER_THRESHOLD = 500;
static const uint8_t MAZE_GOAL_SENSOR_COUNT = 14;
static const uint16_t MAZE_GOAL_STRENGTH = 12500;
static const int16_t MAZE_PROBE_SPEED = 300;
static const int16_t MAZE_TURN_POWER = 500;

static const uint8_t MAZE_MAX_PATH = 160;

static const uint32_t START_RAMP_MS = 500;
static const uint16_t START_MIN_POWER = 250;

static const uint32_t COLOR_SWITCH_CONFIRM_MS = 80;

static const uint16_t LINE_LOST_THRESHOLD = 250;
static const uint16_t LINE_WIDE_THRESHOLD = 11000;
static const uint16_t LINE_MIN_PEAK = 180;
static const uint16_t SENSOR_VALID_RANGE = 250;

static const uint16_t FORK_MARK_THRESHOLD = 650;
static const uint16_t FORK_CENTER_TOLERANCE = 1800;
static const uint32_t FORK_MARK_WINDOW_MS = 600;
static const uint16_t FORK_TURN_BIAS = 140;

static const int16_t LOST_LINE_POWER_MIN = 110;
static const int16_t LOST_LINE_POWER_MAX = 240;

static const uint8_t PWM_BITS = 10;
static const uint16_t PWM_MAX = (1U << PWM_BITS) - 1;
static const uint32_t PWM_FREQ = 20000;
static const uint8_t PWM_CH_L = 0;
static const uint8_t PWM_CH_R = 1;

static const uint8_t MUX_SETTLE_US = 3;

// Derivative low-pass coefficient. Lower = more filtering.
static const float DERIVATIVE_ALPHA = 0.25f;

// dt limits protect the PID from pathological scheduler intervals.
static const float MIN_DT_S = 0.0005f;
static const float MAX_DT_S = 0.020f;

// -----------------------------------------------------------------------------
// PID / robot settings
// -----------------------------------------------------------------------------

float kp = 0.095f;
float ki = 0.000f;
float kd = 0.62f;

int16_t baseSpeed = 420;
int16_t maxSpeed = 850;

bool blackLine = true;
bool mazeModeEnabled = false;
bool invertLeftMotor = false;
bool invertRightMotor = false;

// -----------------------------------------------------------------------------
// UI
// -----------------------------------------------------------------------------

enum RunMode {
  MODE_IDLE,
  MODE_RUNNING,
  MODE_MAZE_RUNNING,
  MODE_CALIBRATING,
  MODE_FAULT
};

enum ManeuverState {
  MANEUVER_NORMAL,
  MANEUVER_UTURN
};

enum MenuItem {
  MENU_BASE_SPEED,
  MENU_MAX_SPEED,
  MENU_KP,
  MENU_KI,
  MENU_KD,
  MENU_LINE_COLOR,
  MENU_DRIVE_MODE,
  MENU_DIAGNOSTICS,
  MENU_SENSOR_VIEW,
  MENU_COUNT
};

struct Button {
  uint8_t pin;
  bool stable;
  bool rawLast;
  uint32_t changedAt;
  bool pressedEvent;
};

Button buttons[] = {
  {PIN_BTN_NEXT, true, true, 0, false},
  {PIN_BTN_PREV, true, true, 0, false},
  {PIN_BTN_ENTER, true, true, 0, false},
  {PIN_BTN_BACK, true, true, 0, false},
  {PIN_BTN_CAL, true, true, 0, false},
  {PIN_BTN_START, true, true, 0, false},
};

static const uint8_t BTN_NEXT = 0;
static const uint8_t BTN_PREV = 1;
static const uint8_t BTN_ENTER = 2;
static const uint8_t BTN_BACK = 3;
static const uint8_t BTN_CAL = 4;
static const uint8_t BTN_START = 5;

RunMode mode = MODE_IDLE;
MenuItem selectedMenu = MENU_BASE_SPEED;
bool editing = false;
bool startButtonLocked = false;
bool settingsDirty = false;

// -----------------------------------------------------------------------------
// Sensor state
// -----------------------------------------------------------------------------

uint16_t sensorRaw[SENSOR_COUNT];
uint16_t sensorNorm[SENSOR_COUNT];
uint16_t sensorMin[SENSOR_COUNT];
uint16_t sensorMax[SENSOR_COUNT];

int32_t lastPosition = ((SENSOR_COUNT - 1) * 1000) / 2;
float lastError = 0.0f;
float integral = 0.0f;
float derivativeFiltered = 0.0f;

bool activeBlackLine = true;
bool colorSwitchPending = false;
uint32_t colorSwitchStartedAt = 0;

uint32_t lastLineSeenAt = 0;
int16_t lastDriveLeft = 0;
int16_t lastDriveRight = 0;

int8_t forkPreference = 0;
uint32_t forkPreferenceUntil = 0;

// Dead-end / U-turn state.
ManeuverState maneuverState = MANEUVER_NORMAL;
uint32_t centeredStrongLineAt = 0;
uint32_t lastSideBranchAt = 0;
uint32_t deadEndCandidateAt = 0;
bool deadEndCandidate = false;
uint32_t uTurnStartedAt = 0;
uint32_t uTurnLineSeenAt = 0;
int8_t uTurnDirection = 1;
uint32_t deadEndCount = 0;


// The maze implementation lives before the diagnostics block, so these
// variables are declared here explicitly and defined later in the file.
extern uint32_t lastFrameAt;
extern uint32_t frameCount;
extern uint32_t controlOverrunCount;
extern uint32_t controlOverrunAt;
extern uint32_t lastFramePeriodUs;
extern uint32_t minFramePeriodUs;
extern uint32_t maxFramePeriodUs;
extern float measuredHz;
extern uint32_t lastControlUs;
extern uint32_t maxControlUs;
extern uint16_t lastLineStrength;
extern uint16_t lastPeakStrength;
extern uint8_t activeSensorCount;
extern float lastConfidence;

// Forward declarations for the maze layer. Keeping these explicit avoids
// relying on Arduino's automatic prototype generation for this larger FSM.
static void setMotors(int16_t left, int16_t right);
static void motorStop();
static void motorEnable();
static uint32_t readSensors();
static void normalizeSensors(bool lineIsBlack);
static bool updateLineMetrics(
  uint16_t *lineStrength,
  uint16_t *peakStrength,
  uint8_t *activeCount,
  float *confidence
);
static int32_t calculatePosition(
  uint16_t *lineStrength,
  uint16_t *peakStrength,
  uint8_t *activeCount,
  float *confidence
);
static bool isColorInversionCandidate(uint16_t currentStrength);
static bool updateLinePolarity(uint16_t *lineStrength);
static int16_t clampSpeed(int16_t value);
static int16_t calculatePidCorrection(float error, float dt);

/*
 * ---------------------------------------------------------------------------
 * Maze solver state
 * ---------------------------------------------------------------------------
 * This is intentionally independent of the normal competition dead-end /
 * U-turn state machine above. Maze mode owns its own junction detection,
 * branch selection, turning, path memory and goal handling.
 */
enum MazeState {
  MAZE_FOLLOW,
  MAZE_PROBE,
  MAZE_TURNING,
  MAZE_FINISHED
};

enum MazeTurn {
  MAZE_TURN_NONE,
  MAZE_TURN_LEFT,
  MAZE_TURN_RIGHT,
  MAZE_TURN_BACK
};

MazeState mazeState = MAZE_FOLLOW;
MazeTurn mazeTurn = MAZE_TURN_NONE;

bool mazeLeftAvailable = false;
bool mazeRightAvailable = false;
bool mazeStraightAvailable = false;

uint32_t mazeProbeStartedAt = 0;
uint32_t mazeTurnStartedAt = 0;
uint32_t mazeLineSeenAt = 0;
uint32_t mazeJunctionLockUntil = 0;
uint32_t mazeDeadEndStartedAt = 0;
uint32_t mazeGoalStartedAt = 0;
uint32_t mazeStartAt = 0;

uint16_t mazeJunctionCount = 0;
uint16_t mazeDeadEndCount = 0;
uint16_t mazePathLength = 0;

char mazePath[MAZE_MAX_PATH + 1];
bool mazeGoalReached = false;
bool mazeFault = false;

static void resetMazeState() {
  mazeState = MAZE_FOLLOW;
  mazeTurn = MAZE_TURN_NONE;

  mazeLeftAvailable = false;
  mazeRightAvailable = false;
  mazeStraightAvailable = false;

  mazeProbeStartedAt = 0;
  mazeTurnStartedAt = 0;
  mazeLineSeenAt = 0;
  mazeJunctionLockUntil = 0;
  mazeDeadEndStartedAt = 0;
  mazeGoalStartedAt = 0;
  mazeStartAt = millis();

  mazeJunctionCount = 0;
  mazeDeadEndCount = 0;
  mazePathLength = 0;

  mazePath[0] = '\0';
  mazeGoalReached = false;
  mazeFault = false;
}

static void mazeAppendMove(char move) {
  if (mazePathLength >= MAZE_MAX_PATH) {
    mazeFault = true;
    mode = MODE_FAULT;
    motorStop();
    Serial.println(F("MAZE FAULT: path memory full."));
    return;
  }

  mazePath[mazePathLength++] = move;
  mazePath[mazePathLength] = '\0';
}

static char mazeReduceTriple(char a, char b, char c) {
  if (b != 'B') return 0;

  // Relative turn angles:
  // L=-90, S=0, R=+90, B=180.
  int angle = 0;

  auto turnAngle = [](char x) -> int {
    if (x == 'L') return 270;
    if (x == 'R') return 90;
    if (x == 'B') return 180;
    return 0;
  };

  angle = (turnAngle(a) + 180 + turnAngle(c)) % 360;

  if (angle == 0) return 'S';
  if (angle == 90) return 'R';
  if (angle == 180) return 'B';
  if (angle == 270) return 'L';

  return 0;
}

static void mazeSimplifyPath() {
  bool changed = true;

  while (changed && mazePathLength >= 3) {
    changed = false;

    for (uint16_t i = 0; i + 2 < mazePathLength; i++) {
      char replacement = mazeReduceTriple(
        mazePath[i],
        mazePath[i + 1],
        mazePath[i + 2]
      );

      if (replacement == 0) continue;

      mazePath[i] = replacement;

      for (uint16_t j = i + 1; j + 2 < mazePathLength; j++) {
        mazePath[j] = mazePath[j + 2];
      }

      mazePathLength -= 2;
      mazePath[mazePathLength] = '\0';

      changed = true;
      break;
    }
  }
}

static void mazePrintPath() {
  Serial.print(F("MAZE PATH="));
  Serial.println(mazePath);
}

static bool mazeOuterBranch(bool left) {
  const uint8_t start = left ? 0 : SENSOR_COUNT - MAZE_BRANCH_SENSORS;

  uint8_t count = 0;

  for (uint8_t n = 0; n < MAZE_BRANCH_SENSORS; n++) {
    const uint8_t i = start + n;

    if (sensorNorm[i] >= MAZE_BRANCH_THRESHOLD) {
      count++;
    }
  }

  return count >= 2;
}

static bool mazeCenterOnLine() {
  const uint8_t centerLeft = SENSOR_COUNT / 2 - 1;
  const uint8_t centerRight = SENSOR_COUNT / 2;

  return sensorNorm[centerLeft] >= MAZE_CENTER_THRESHOLD ||
         sensorNorm[centerRight] >= MAZE_CENTER_THRESHOLD;
}

static bool mazeWideJunctionCandidate() {
  const bool left = mazeOuterBranch(true);
  const bool right = mazeOuterBranch(false);

  // A side branch is the strongest junction indication. A very wide line
  // with multiple active sensors is also treated as a candidate, allowing
  // T/cross intersections whose branch geometry spans the array.
  return left ||
         right ||
         (activeSensorCount >= 8 &&
          lastLineStrength >= 3500);
}

static bool mazeGoalCandidate() {
  return millis() - mazeStartAt >= MAZE_START_IGNORE_MS &&
         activeSensorCount >= MAZE_GOAL_SENSOR_COUNT &&
         lastLineStrength >= MAZE_GOAL_STRENGTH;
}

static void mazeBeginTurn(MazeTurn turn) {
  mazeTurn = turn;
  mazeState = MAZE_TURNING;
  mazeTurnStartedAt = millis();
  mazeLineSeenAt = 0;

  integral = 0.0f;
  derivativeFiltered = 0.0f;
  lastError = 0.0f;

  if (turn == MAZE_TURN_LEFT) {
    mazeAppendMove('L');
  } else if (turn == MAZE_TURN_RIGHT) {
    mazeAppendMove('R');
  } else if (turn == MAZE_TURN_BACK) {
    mazeAppendMove('B');
    mazeDeadEndCount++;
  }

  if (mode == MODE_FAULT) return;

  Serial.print(F("MAZE TURN "));
  Serial.println(
    turn == MAZE_TURN_LEFT ? F("LEFT") :
    turn == MAZE_TURN_RIGHT ? F("RIGHT") :
    F("BACK")
  );
}

static void mazeChooseAndTurn() {
  // Left-hand rule:
  // LEFT > STRAIGHT > RIGHT > BACK.
  // The recorded L/S/R/B path is reduced after every backtrack.
  if (mazeLeftAvailable) {
    mazeBeginTurn(MAZE_TURN_LEFT);
  } else if (mazeStraightAvailable) {
    mazeAppendMove('S');
    mazeState = MAZE_FOLLOW;
    mazeJunctionLockUntil =
      millis() + MAZE_JUNCTION_DEBOUNCE_MS;

    Serial.println(F("MAZE CHOICE STRAIGHT"));
  } else if (mazeRightAvailable) {
    mazeBeginTurn(MAZE_TURN_RIGHT);
  } else {
    mazeBeginTurn(MAZE_TURN_BACK);
  }
}

static bool mazeServiceTurn(
  uint16_t lineStrength
) {
  if (mazeState != MAZE_TURNING) {
    return false;
  }

  const uint32_t now = millis();
  const uint32_t elapsed =
    now - mazeTurnStartedAt;

  if (elapsed < MAZE_TURN_CLEAR_MS) {
    if (mazeTurn == MAZE_TURN_LEFT) {
      setMotors(-MAZE_TURN_POWER, MAZE_TURN_POWER);
    } else {
      setMotors(MAZE_TURN_POWER, -MAZE_TURN_POWER);
    }

    return true;
  }

  const bool centerLine =
    mazeCenterOnLine() &&
    lineStrength >= LINE_LOST_THRESHOLD;

  if (centerLine) {
    if (mazeLineSeenAt == 0) {
      mazeLineSeenAt = now;
    }

    if (now - mazeLineSeenAt >= MAZE_LINE_REACQUIRE_MS) {
      setMotors(0, 0);
      motorEnable();

      mazeState = MAZE_FOLLOW;
      mazeTurn = MAZE_TURN_NONE;
      mazeJunctionLockUntil =
        now + MAZE_JUNCTION_DEBOUNCE_MS;

      if (mazePathLength > 0 &&
          mazePath[mazePathLength - 1] == 'B') {
        mazeSimplifyPath();
      }

      Serial.print(F("MAZE LINE REACQUIRED path="));
      Serial.println(mazePath);

      return true;
    }
  } else {
    mazeLineSeenAt = 0;
  }

  if (elapsed >= MAZE_TURN_TIMEOUT_MS) {
    motorStop();
    mode = MODE_FAULT;
    mazeFault = true;

    Serial.println(
      F("MAZE FAULT: turn timeout / line not reacquired.")
    );

    return true;
  }

  if (mazeTurn == MAZE_TURN_LEFT) {
    setMotors(-MAZE_TURN_POWER, MAZE_TURN_POWER);
  } else {
    setMotors(MAZE_TURN_POWER, -MAZE_TURN_POWER);
  }

  return true;
}

static void mazeFinish() {
  motorStop();

  mazeState = MAZE_FINISHED;
  mazeGoalReached = true;

  // Remove dead-end excursions before presenting the final route.
  mazeSimplifyPath();

  Serial.println(F("===================================="));
  Serial.println(F("MAZE SOLVED"));
  mazePrintPath();
  Serial.print(F("Junctions="));
  Serial.println(mazeJunctionCount);
  Serial.print(F("DeadEnds="));
  Serial.println(mazeDeadEndCount);
  Serial.print(F("PathLength="));
  Serial.println(mazePathLength);
  Serial.println(F("===================================="));
}

static void runMazeFrame() {
  const uint32_t frameStart = micros();
  const uint32_t nowUs = frameStart;

  if (lastFrameAt != 0) {
    lastFramePeriodUs = nowUs - lastFrameAt;

    if (lastFramePeriodUs > 0) {
      measuredHz =
        1000000.0f / (float)lastFramePeriodUs;
    }

    if (lastFramePeriodUs < minFramePeriodUs) {
      minFramePeriodUs = lastFramePeriodUs;
    }

    if (lastFramePeriodUs > maxFramePeriodUs) {
      maxFramePeriodUs = lastFramePeriodUs;
    }

    if (lastFramePeriodUs > CONTROL_PERIOD_US * 2) {
      controlOverrunCount++;
      controlOverrunAt = millis();
    }
  }

  lastFrameAt = nowUs;
  frameCount++;

  const uint32_t scanUs = readSensors();

  if (scanUs >= CONTROL_PERIOD_US) {
    controlOverrunCount++;
    controlOverrunAt = millis();
  }

  normalizeSensors(activeBlackLine);

  uint16_t lineStrength = 0;
  uint16_t peakStrength = 0;
  uint8_t activeCount = 0;
  float confidence = 0.0f;

  int32_t position = calculatePosition(
    &lineStrength,
    &peakStrength,
    &activeCount,
    &confidence
  );

  lastLineStrength = lineStrength;
  lastPeakStrength = peakStrength;
  activeSensorCount = activeCount;
  lastConfidence = confidence;

  const uint32_t nowMs = millis();

  const bool inversionCandidate =
    isColorInversionCandidate(lineStrength);

  if (updateLinePolarity(&lineStrength)) {
    uint32_t weightedSum = 0;
    uint32_t sum = 0;

    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
      weightedSum +=
        (uint32_t)sensorNorm[i] *
        (uint32_t)(i * 1000);
      sum += sensorNorm[i];
    }

    if (sum > 0) {
      position = weightedSum / sum;
      lastPosition = position;
    }

    updateLineMetrics(
      &lineStrength,
      &peakStrength,
      &activeCount,
      &confidence
    );

    lastLineStrength = lineStrength;
    lastPeakStrength = peakStrength;
    activeSensorCount = activeCount;
    lastConfidence = confidence;
  }

  if (mazeState == MAZE_FINISHED) {
    motorStop();
    lastControlUs = micros() - frameStart;
    return;
  }

  if (mazeGoalCandidate()) {
    if (mazeGoalStartedAt == 0) {
      mazeGoalStartedAt = nowMs;
    }

    if (nowMs - mazeGoalStartedAt >= MAZE_GOAL_CONFIRM_MS) {
      mazeFinish();
      lastControlUs = micros() - frameStart;
      return;
    }
  } else {
    mazeGoalStartedAt = 0;
  }

  if (mazeServiceTurn(lineStrength)) {
    lastControlUs = micros() - frameStart;
    if (lastControlUs > maxControlUs) {
      maxControlUs = lastControlUs;
    }
    return;
  }

  if (mazeState == MAZE_PROBE) {
    // Move beyond the junction so the center sensors can tell us whether
    // the straight branch actually continues.
    setMotors(MAZE_PROBE_SPEED, MAZE_PROBE_SPEED);

    if (nowMs - mazeProbeStartedAt >= MAZE_PROBE_MS) {
      mazeStraightAvailable =
        mazeCenterOnLine() &&
        lineStrength >= LINE_LOST_THRESHOLD;

      mazeJunctionCount++;

      mazeChooseAndTurn();

      if (mode != MODE_FAULT &&
          mazeState != MAZE_TURNING) {
        mazeJunctionLockUntil =
          nowMs + MAZE_JUNCTION_DEBOUNCE_MS;
      }

      lastControlUs = micros() - frameStart;
      if (lastControlUs > maxControlUs) {
        maxControlUs = lastControlUs;
      }
      return;
    }

    lastControlUs = micros() - frameStart;
    if (lastControlUs > maxControlUs) {
      maxControlUs = lastControlUs;
    }
    return;
  }

  if (nowMs >= mazeJunctionLockUntil &&
      mazeWideJunctionCandidate()) {

    mazeLeftAvailable = mazeOuterBranch(true);
    mazeRightAvailable = mazeOuterBranch(false);

    // At the first detection the center line cannot distinguish a T from a
    // cross. The short forward probe resolves that ambiguity.
    mazeStraightAvailable = true;
    mazeProbeStartedAt = nowMs;
    mazeState = MAZE_PROBE;

    setMotors(
      MAZE_PROBE_SPEED,
      MAZE_PROBE_SPEED
    );

    lastControlUs = micros() - frameStart;
    if (lastControlUs > maxControlUs) {
      maxControlUs = lastControlUs;
    }
    return;
  }

  const bool linePresent =
    peakStrength >= LINE_MIN_PEAK &&
    lineStrength >= LINE_LOST_THRESHOLD &&
    confidence >= 0.15f;

  // Separate maze dead-end detection. Do not reuse the competition-mode
  // dead-end state machine: maze mode needs the B path record.
  if (!linePresent) {
    if (mazeDeadEndStartedAt == 0) {
      mazeDeadEndStartedAt = nowMs;
    }

    if (nowMs - mazeDeadEndStartedAt >=
        MAZE_DEAD_END_CONFIRM_MS) {

      mazeDeadEndStartedAt = 0;
      mazeBeginTurn(MAZE_TURN_BACK);

      lastControlUs = micros() - frameStart;
      if (lastControlUs > maxControlUs) {
        maxControlUs = lastControlUs;
      }
      return;
    }

    // Brief line gaps: continue with the previous command.
    setMotors(lastDriveLeft, lastDriveRight);

    lastControlUs = micros() - frameStart;
    if (lastControlUs > maxControlUs) {
      maxControlUs = lastControlUs;
    }
    return;
  }

  mazeDeadEndStartedAt = 0;
  lastLineSeenAt = nowMs;

  // Normal PID line following inside the maze.
  const float dt =
    constrain(
      (lastFramePeriodUs > 0
        ? lastFramePeriodUs * 1e-6f
        : CONTROL_PERIOD_US * 1e-6f),
      MIN_DT_S,
      MAX_DT_S
    );

  const int32_t center =
    (SENSOR_COUNT - 1) * 1000 / 2;

  const float error =
    (float)position - (float)center;

  const int16_t correction =
    calculatePidCorrection(error, dt);

  const int16_t mazeBase =
    min(baseSpeed, (int16_t)(maxSpeed * 0.75f));

  int16_t left =
    (int16_t)(mazeBase + correction);

  int16_t right =
    (int16_t)(mazeBase - correction);

  const int16_t magnitude =
    max(abs(left), abs(right));

  if (magnitude > maxSpeed) {
    const float scale =
      (float)maxSpeed / (float)magnitude;
    left = (int16_t)(left * scale);
    right = (int16_t)(right * scale);
  }

  left = clampSpeed(left);
  right = clampSpeed(right);

  setMotors(left, right);

  lastDriveLeft = left;
  lastDriveRight = right;
  lastError = error;

  lastControlUs = micros() - frameStart;

  if (lastControlUs > maxControlUs) {
    maxControlUs = lastControlUs;
  }
}


uint32_t lastStartPressedAt = 0;
uint32_t lastStopPressedAt = 0;
uint32_t calibrationStartedAt = 0;

// -----------------------------------------------------------------------------
// Diagnostics
// -----------------------------------------------------------------------------

uint32_t lastFrameAt = 0;
uint32_t frameCount = 0;
uint32_t controlOverrunCount = 0;
uint32_t sensorFaultCount = 0;

uint32_t lastFramePeriodUs = 0;
uint32_t minFramePeriodUs = UINT32_MAX;
uint32_t maxFramePeriodUs = 0;

uint32_t lastSensorScanUs = 0;
uint32_t minSensorScanUs = UINT32_MAX;
uint32_t maxSensorScanUs = 0;

uint32_t lastControlUs = 0;
uint32_t maxControlUs = 0;

uint32_t lastTelemetryAt = 0;
uint32_t lastUiAt = 0;
uint32_t lastButtonAt = 0;

float measuredHz = 0.0f;

uint16_t lastLineStrength = 0;
uint16_t lastPeakStrength = 0;
uint8_t activeSensorCount = 0;
float lastConfidence = 0.0f;

uint32_t controlOverrunAt = 0;

// -----------------------------------------------------------------------------
// Dead-end detection / U-turn helpers
// -----------------------------------------------------------------------------

static bool detectSideBranch() {
  uint8_t leftCount = 0;
  uint8_t rightCount = 0;

  for (uint8_t i = 0; i < DEAD_END_SIDE_SENSORS; i++) {
    if (sensorNorm[i] >= DEAD_END_SIDE_THRESHOLD) {
      leftCount++;
    }

    const uint8_t rightIndex =
      SENSOR_COUNT - 1 - i;

    if (sensorNorm[rightIndex] >= DEAD_END_SIDE_THRESHOLD) {
      rightCount++;
    }
  }

  return leftCount > 0 || rightCount > 0;
}

static bool deadEndConditionsReady(
  uint32_t nowMs
) {
  // The strong/centered condition must come from the last valid line frame.
  // At this point the current frame is already line-less, so using the
  // current line strength here would make dead-end detection impossible.
  const bool recentCenteredStrong =
    centeredStrongLineAt != 0 &&
    nowMs - centeredStrongLineAt <=
      DEAD_END_MEMORY_MS;

  const bool recentSideBranch =
    lastSideBranchAt != 0 &&
    nowMs - lastSideBranchAt <=
      DEAD_END_MEMORY_MS;

  return recentCenteredStrong &&
         !recentSideBranch;
}

static void resetManeuverState() {
  maneuverState = MANEUVER_NORMAL;
  deadEndCandidate = false;
  deadEndCandidateAt = 0;
  uTurnStartedAt = 0;
  uTurnLineSeenAt = 0;
  uTurnDirection = 1;
}

static void beginUTurn(int8_t direction) {
  maneuverState = MANEUVER_UTURN;
  uTurnStartedAt = millis();
  uTurnLineSeenAt = 0;
  uTurnDirection =
    direction < 0 ? -1 : 1;

  // A U-turn must start from a clean PID state.
  integral = 0.0f;
  derivativeFiltered = 0.0f;
  lastError = 0.0f;
  forkPreference = 0;
  forkPreferenceUntil = 0;

  deadEndCandidate = false;
  deadEndCandidateAt = 0;
  deadEndCount++;

  Serial.println(
    F("DEAD END: confirmed, starting 180-degree turn.")
  );
}

static bool serviceUTurn(
  bool linePresent,
  float confidence,
  uint32_t nowMs
) {
  if (maneuverState != MANEUVER_UTURN) {
    return false;
  }

  const uint32_t elapsed =
    nowMs - uTurnStartedAt;

  // The minimum rotation prevents immediately reacquiring the old line
  // and cancelling the maneuver before the robot has turned enough.
  if (elapsed < UTURN_MIN_MS) {
    setMotors(
      -uTurnDirection * UTURN_POWER,
      uTurnDirection * UTURN_POWER
    );

    lastDriveLeft =
      -uTurnDirection * UTURN_POWER;

    lastDriveRight =
      uTurnDirection * UTURN_POWER;

    return true;
  }

  if (linePresent && confidence >= 0.25f) {
    if (uTurnLineSeenAt == 0) {
      uTurnLineSeenAt = nowMs;
    }

    if (nowMs - uTurnLineSeenAt >=
        UTURN_REACQUIRE_MS) {

      setMotors(0, 0);
      motorEnable();

      maneuverState = MANEUVER_NORMAL;
      lastLineSeenAt = nowMs;

      Serial.println(
        F("U-TURN: line reacquired.")
      );

      return true;
    }
  } else {
    uTurnLineSeenAt = 0;
  }

  if (elapsed >= UTURN_MAX_MS) {
    motorStop();
    mode = MODE_FAULT;

    Serial.println(
      F("U-TURN FAILED: line not reacquired.")
    );

    return true;
  }

  setMotors(
    -uTurnDirection * UTURN_POWER,
    uTurnDirection * UTURN_POWER
  );

  lastDriveLeft =
    -uTurnDirection * UTURN_POWER;

  lastDriveRight =
    uTurnDirection * UTURN_POWER;

  return true;
}

// -----------------------------------------------------------------------------
// Peripherals
// -----------------------------------------------------------------------------

Preferences prefs;
Adafruit_SSD1306 display(128, 64, &Wire, -1);
bool displayReady = false;

// -----------------------------------------------------------------------------
// Utility
// -----------------------------------------------------------------------------

static int16_t clampSpeed(int16_t value) {
  if (value > maxSpeed) return maxSpeed;
  if (value < -maxSpeed) return -maxSpeed;
  return value;
}

static void markSettingsDirty() {
  settingsDirty = true;
}

static bool calibrationLooksValid() {
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    if (sensorMax[i] <= sensorMin[i]) return false;
    if ((sensorMax[i] - sensorMin[i]) < SENSOR_VALID_RANGE) return false;
  }
  return true;
}

// -----------------------------------------------------------------------------
// Preferences
// -----------------------------------------------------------------------------

static void saveSettings() {
  prefs.begin("lf-v2", false);

  prefs.putFloat("kp", kp);
  prefs.putFloat("ki", ki);
  prefs.putFloat("kd", kd);
  prefs.putShort("base", baseSpeed);
  prefs.putShort("max", maxSpeed);
  prefs.putBool("black", blackLine);
  prefs.putBool("maze", mazeModeEnabled);
  prefs.putBool("calOK", calibrationLooksValid());

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    char keyMin[8];
    char keyMax[8];

    snprintf(keyMin, sizeof(keyMin), "mn%u", i);
    snprintf(keyMax, sizeof(keyMax), "mx%u", i);

    prefs.putUShort(keyMin, sensorMin[i]);
    prefs.putUShort(keyMax, sensorMax[i]);
  }

  prefs.end();
  settingsDirty = false;
}

static void loadSettings() {
  prefs.begin("lf-v2", true);

  kp = prefs.getFloat("kp", kp);
  ki = prefs.getFloat("ki", ki);
  kd = prefs.getFloat("kd", kd);

  baseSpeed = prefs.getShort("base", baseSpeed);
  maxSpeed = prefs.getShort("max", maxSpeed);

  blackLine = prefs.getBool("black", blackLine);
  mazeModeEnabled = prefs.getBool("maze", mazeModeEnabled);

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    char keyMin[8];
    char keyMax[8];

    snprintf(keyMin, sizeof(keyMin), "mn%u", i);
    snprintf(keyMax, sizeof(keyMax), "mx%u", i);

    sensorMin[i] = prefs.getUShort(keyMin, 300);
    sensorMax[i] = prefs.getUShort(keyMax, 3800);
  }

  prefs.end();

  kp = max(0.0f, kp);
  ki = max(0.0f, ki);
  kd = max(0.0f, kd);

  baseSpeed = constrain(baseSpeed, 0, (int16_t)PWM_MAX);
  maxSpeed = constrain(maxSpeed, 100, (int16_t)PWM_MAX);

  if (baseSpeed > maxSpeed) {
    baseSpeed = maxSpeed;
  }

  activeBlackLine = blackLine;
}

static void resetCalibrationBounds() {
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    sensorMin[i] = 4095;
    sensorMax[i] = 0;
  }
}

// -----------------------------------------------------------------------------
// Motor driver
// -----------------------------------------------------------------------------

static bool setupMotors() {
  pinMode(PIN_MOTOR_L_IN1, OUTPUT);
  pinMode(PIN_MOTOR_L_IN2, OUTPUT);
  pinMode(PIN_MOTOR_R_IN1, OUTPUT);
  pinMode(PIN_MOTOR_R_IN2, OUTPUT);
  pinMode(PIN_MOTOR_STBY, OUTPUT);

  bool leftOK = ledcAttachChannel(
    PIN_MOTOR_L_PWM,
    PWM_FREQ,
    PWM_BITS,
    PWM_CH_L
  );

  bool rightOK = ledcAttachChannel(
    PIN_MOTOR_R_PWM,
    PWM_FREQ,
    PWM_BITS,
    PWM_CH_R
  );

  digitalWrite(PIN_MOTOR_STBY, LOW);

  return leftOK && rightOK;
}

static void setOneMotor(
  uint8_t pwmPin,
  uint8_t in1,
  uint8_t in2,
  int16_t speed,
  bool invert
) {
  if (invert) {
    speed = -speed;
  }

  speed = constrain(
    speed,
    -(int16_t)PWM_MAX,
    (int16_t)PWM_MAX
  );

  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    ledcWrite(pwmPin, speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    ledcWrite(pwmPin, -speed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(pwmPin, 0);
  }
}

static void setMotors(int16_t left, int16_t right) {
  left = clampSpeed(left);
  right = clampSpeed(right);

  setOneMotor(
    PIN_MOTOR_L_PWM,
    PIN_MOTOR_L_IN1,
    PIN_MOTOR_L_IN2,
    left,
    invertLeftMotor
  );

  setOneMotor(
    PIN_MOTOR_R_PWM,
    PIN_MOTOR_R_IN1,
    PIN_MOTOR_R_IN2,
    right,
    invertRightMotor
  );
}

static void motorStop() {
  setMotors(0, 0);
  digitalWrite(PIN_MOTOR_STBY, LOW);

  integral = 0.0f;
  derivativeFiltered = 0.0f;
}

static void motorEnable() {
  digitalWrite(PIN_MOTOR_STBY, HIGH);
}

// -----------------------------------------------------------------------------
// Sensor acquisition
// -----------------------------------------------------------------------------

static void setupSensors() {
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  pinMode(PIN_SENSOR_MUX_OUT, INPUT);

  pinMode(PIN_SENSOR_S0, OUTPUT);
  pinMode(PIN_SENSOR_S1, OUTPUT);
  pinMode(PIN_SENSOR_S2, OUTPUT);
  pinMode(PIN_SENSOR_S3, OUTPUT);
}

static void selectSensorChannel(uint8_t channel) {
  digitalWrite(PIN_SENSOR_S0, (channel & 0x01) ? HIGH : LOW);
  digitalWrite(PIN_SENSOR_S1, (channel & 0x02) ? HIGH : LOW);
  digitalWrite(PIN_SENSOR_S2, (channel & 0x04) ? HIGH : LOW);
  digitalWrite(PIN_SENSOR_S3, (channel & 0x08) ? HIGH : LOW);

  delayMicroseconds(MUX_SETTLE_US);
}

static uint32_t readSensors() {
  const uint32_t startUs = micros();

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    selectSensorChannel(i);
    sensorRaw[i] = analogRead(PIN_SENSOR_MUX_OUT);
  }

  const uint32_t elapsed = micros() - startUs;

  lastSensorScanUs = elapsed;

  if (elapsed < minSensorScanUs) minSensorScanUs = elapsed;
  if (elapsed > maxSensorScanUs) maxSensorScanUs = elapsed;

  return elapsed;
}

// -----------------------------------------------------------------------------
// Calibration
// -----------------------------------------------------------------------------

static void updateCalibration() {
  readSensors();

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    if (sensorRaw[i] < sensorMin[i]) {
      sensorMin[i] = sensorRaw[i];
    }

    if (sensorRaw[i] > sensorMax[i]) {
      sensorMax[i] = sensorRaw[i];    }  }
}

static void beginCalibration() {
  motorStop();

  mode = MODE_CALIBRATING;
  calibrationStartedAt = millis();

  resetCalibrationBounds();

  Serial.println(F("V2 calibration started."));
  Serial.println(F("Move the sensor array across both line and background."));
}

static void finishCalibration() {
  if (!calibrationLooksValid()) {
    mode = MODE_FAULT;
    sensorFaultCount++;
    motorStop();

    Serial.println(F("CALIBRATION FAILED: insufficient sensor range."));
    return;
  }

  activeBlackLine = blackLine;
  saveSettings();

  mode = MODE_IDLE;
  motorStop();

  Serial.println(F("Calibration successful."));
}

// -----------------------------------------------------------------------------
// Sensor processing
// -----------------------------------------------------------------------------

static void normalizeSensors(bool lineIsBlack) {
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    uint16_t lo = sensorMin[i];
    uint16_t hi = sensorMax[i];
    uint16_t raw = sensorRaw[i];

    if (hi <= lo || (hi - lo) < SENSOR_VALID_RANGE) {
      sensorNorm[i] = 0;
      continue;
    }

    raw = constrain(raw, lo, hi);

    uint32_t value =
      ((uint32_t)(raw - lo) * 1000UL) /
      (uint32_t)(hi - lo);

    if (value > 1000) value = 1000;

    sensorNorm[i] =
      lineIsBlack
        ? (uint16_t)value
        : (uint16_t)(1000 - value);
  }
}

static bool updateLineMetrics(
  uint16_t *lineStrength,
  uint16_t *peakStrength,
  uint8_t *activeCount,
  float *confidence
) {
  uint32_t sum = 0;
  uint16_t peak = 0;
  uint8_t count = 0;

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    const uint16_t value = sensorNorm[i];

    sum += value;

    if (value > peak) {
      peak = value;
    }

    if (value >= 300) {
      count++;
    }
  }

  *lineStrength = (uint16_t)min(sum, 65535UL);
  *peakStrength = peak;
  *activeCount = count;

  // Confidence combines peak strength, total energy and sensor occupancy.
  // It is deliberately simple and cheap enough for the control path.
  float peakScore =
    min(1.0f, peak / 1000.0f);

  float sumScore =
    min(1.0f, sum / 5000.0f);

  float occupancyScore =
    min(1.0f, count / 5.0f);

  *confidence =
    0.50f * peakScore +
    0.30f * sumScore +
    0.20f * occupancyScore;

  return peak >= LINE_MIN_PEAK;
}

static int32_t calculatePosition(
  uint16_t *lineStrength,
  uint16_t *peakStrength,
  uint8_t *activeCount,
  float *confidence
) {
  uint32_t weightedSum = 0;
  uint32_t sum = 0;

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    const uint16_t value = sensorNorm[i];

    weightedSum +=
      (uint32_t)value * (uint32_t)(i * 1000);

    sum += value;
  }

  updateLineMetrics(
    lineStrength,
    peakStrength,
    activeCount,
    confidence
  );

  if (sum < LINE_LOST_THRESHOLD) {
    return lastPosition;
  }

  int32_t position =
    (int32_t)(weightedSum / sum);

  lastPosition = position;

  return position;
}

// -----------------------------------------------------------------------------
// Color inversion
// -----------------------------------------------------------------------------

static bool isColorInversionCandidate(
  uint16_t currentStrength
) {
  if (currentStrength < LINE_WIDE_THRESHOLD) {
    return false;
  }

  const uint16_t invertedStrength =
    (uint16_t)(SENSOR_COUNT * 1000U) -
    min<uint16_t>(
      currentStrength,
      SENSOR_COUNT * 1000U
    );

  return invertedStrength >= LINE_LOST_THRESHOLD &&
         invertedStrength < LINE_WIDE_THRESHOLD;
}

static bool updateLinePolarity(
  uint16_t *lineStrength
) {
  if (!isColorInversionCandidate(*lineStrength)) {
    colorSwitchPending = false;
    return false;
  }

  const uint32_t now = millis();

  if (!colorSwitchPending) {
    colorSwitchPending = true;
    colorSwitchStartedAt = now;
    return false;
  }

  if (now - colorSwitchStartedAt <
      COLOR_SWITCH_CONFIRM_MS) {
    return false;
  }

  activeBlackLine = !activeBlackLine;
  colorSwitchPending = false;

  normalizeSensors(activeBlackLine);

  uint32_t sum = 0;

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    sum += sensorNorm[i];
  }

  *lineStrength =
    (uint16_t)min(sum, 65535UL);

  return true;
}

// -----------------------------------------------------------------------------
// Buttons
// -----------------------------------------------------------------------------

static void setupButtons() {
  for (Button &button : buttons) {
    if (button.pin == 34 || button.pin == 35) {
      // GPIO34/35 are input-only and have no internal pull-up.
      // External pull-ups are required by the hardware.
      pinMode(button.pin, INPUT);
    } else {
      pinMode(button.pin, INPUT_PULLUP);
    }

    button.stable = digitalRead(button.pin);
    button.rawLast = button.stable;
    button.changedAt = millis();
  }
}

static void updateButtons() {
  const uint32_t now = millis();

  if (now - lastButtonAt < BUTTON_PERIOD_MS) {
    return;
  }

  lastButtonAt = now;

  for (Button &button : buttons) {
    button.pressedEvent = false;

    const bool raw = digitalRead(button.pin);

    if (raw != button.rawLast) {
      button.rawLast = raw;
      button.changedAt = now;
    }

    if (now - button.changedAt >= 25 &&
        raw != button.stable) {

      const bool oldState = button.stable;
      button.stable = raw;

      if (oldState == HIGH &&
          button.stable == LOW) {
        button.pressedEvent = true;
      }
    }
  }
}

static bool pressed(uint8_t index) {
  return buttons[index].pressedEvent;
}

// -----------------------------------------------------------------------------
// Settings menu
// -----------------------------------------------------------------------------

static void adjustSelected(int8_t direction) {
  switch (selectedMenu) {

    case MENU_BASE_SPEED:
      baseSpeed = constrain(
        baseSpeed + direction * 20,
        0,
        (int16_t)PWM_MAX
      );

      if (baseSpeed > maxSpeed) {
        baseSpeed = maxSpeed;
      }

      markSettingsDirty();
      break;

    case MENU_MAX_SPEED:
      maxSpeed = constrain(
        maxSpeed + direction * 20,
        100,
        (int16_t)PWM_MAX
      );

      if (baseSpeed > maxSpeed) {
        baseSpeed = maxSpeed;
      }

      markSettingsDirty();
      break;

    case MENU_KP:
      kp = max(
        0.0f,
        kp + direction * 0.005f
      );

      markSettingsDirty();
      break;

    case MENU_KI:
      ki = max(
        0.0f,
        ki + direction * 0.0001f
      );

      markSettingsDirty();
      break;

    case MENU_KD:
      kd = max(
        0.0f,
        kd + direction * 0.025f
      );

      markSettingsDirty();
      break;

    case MENU_LINE_COLOR:
      if (direction != 0) {
        blackLine = !blackLine;
        activeBlackLine = blackLine;
        markSettingsDirty();
      }
      break;

    case MENU_DRIVE_MODE:
      if (direction != 0) {
        mazeModeEnabled = !mazeModeEnabled;
        markSettingsDirty();
      }
      break;

    default:
      break;
  }
}

static const char *menuName(MenuItem item) {
  switch (item) {
    case MENU_BASE_SPEED: return "Base speed";
    case MENU_MAX_SPEED: return "Max speed";
    case MENU_KP: return "Kp";
    case MENU_KI: return "Ki";
    case MENU_KD: return "Kd";
    case MENU_LINE_COLOR: return "Line color";
    case MENU_DRIVE_MODE: return "Drive mode";
    case MENU_DIAGNOSTICS: return "Diagnostics";
    case MENU_SENSOR_VIEW: return "Sensor view";
    default: return "";
  }
}

static String menuValueText(MenuItem item) {
  switch (item) {
    case MENU_BASE_SPEED:
      return String(baseSpeed);

    case MENU_MAX_SPEED:
      return String(maxSpeed);

    case MENU_KP:
      return String(kp, 3);

    case MENU_KI:
      return String(ki, 4);

    case MENU_KD:
      return String(kd, 3);

    case MENU_LINE_COLOR:
      return blackLine ? "Black" : "White";

    case MENU_DRIVE_MODE:
      return mazeModeEnabled ? "Maze" : "Line";

    case MENU_DIAGNOSTICS:
      return "Enter";

    case MENU_SENSOR_VIEW:
      return "Enter";

    default:
      return "";
  }
}

static MenuItem menuOffset(
  MenuItem item,
  int8_t offset
) {
  int8_t index =
    (int8_t)item + offset;

  while (index < 0) {
    index += MENU_COUNT;
  }

  while (index >= MENU_COUNT) {
    index -= MENU_COUNT;
  }

  return (MenuItem)index;
}

static void handleButtons() {
  updateButtons();

  if (pressed(BTN_START) &&
      !startButtonLocked) {

    editing = false;
    startButtonLocked = true;

    if (mode == MODE_RUNNING ||
        mode == MODE_MAZE_RUNNING) {

      mode = MODE_IDLE;
      lastStopPressedAt = millis();

      motorStop();

      if (settingsDirty) {
        saveSettings();
      }

    } else if (mode == MODE_IDLE) {

      if (!calibrationLooksValid()) {
        mode = MODE_FAULT;
        sensorFaultCount++;

        Serial.println(
          F("START BLOCKED: calibration invalid.")
        );

        motorStop();

      } else {

        lastStartPressedAt = millis();
        lastLineSeenAt = millis();

        lastPosition =
          ((SENSOR_COUNT - 1) * 1000) / 2;

        lastError = 0.0f;
        integral = 0.0f;
        derivativeFiltered = 0.0f;

        forkPreference = 0;
        forkPreferenceUntil = 0;
        colorSwitchPending = false;

        centeredStrongLineAt = 0;
        lastSideBranchAt = 0;
        deadEndCandidateAt = 0;
        deadEndCandidate = false;
        deadEndCount = 0;
        resetManeuverState();

        frameCount = 0;
        controlOverrunCount = 0;

        minFramePeriodUs = UINT32_MAX;
        maxFramePeriodUs = 0;

        minSensorScanUs = UINT32_MAX;
        maxSensorScanUs = 0;

        maxControlUs = 0;
        lastFrameAt = micros();

        resetMazeState();

        if (mazeModeEnabled) {
          mode = MODE_MAZE_RUNNING;
          Serial.println(F("V2 MAZE RUN START"));
        } else {
          mode = MODE_RUNNING;
          Serial.println(F("V2 LINE RUN START"));
        }

        motorEnable();
      }
    } else if (mode == MODE_FAULT) {

      // START acknowledges a recoverable fault.
      if (calibrationLooksValid()) {
        mode = MODE_IDLE;
        Serial.println(F("Fault acknowledged."));
      }
    }
  }

  // Calibration must remain available after a calibration-related fault.
  if (pressed(BTN_CAL) &&
      (mode == MODE_IDLE || mode == MODE_FAULT)) {
    beginCalibration();
  }

  if (buttons[BTN_START].stable == HIGH) {
    startButtonLocked = false;
  }

  if (mode != MODE_IDLE) {
    return;
  }

  if (pressed(BTN_ENTER)) {
    if (selectedMenu == MENU_DIAGNOSTICS ||
        selectedMenu == MENU_SENSOR_VIEW) {
      editing = true;
    } else {      editing = !editing;
      if (!editing && settingsDirty) {
        saveSettings();
      }
    }
  }

  if (pressed(BTN_BACK)) {
    if (editing) {
      editing = false;

      if (settingsDirty) {
        saveSettings();
      }
    }
  }

  if (pressed(BTN_NEXT)) {
    if (editing) {
      if (selectedMenu != MENU_DIAGNOSTICS &&
          selectedMenu != MENU_SENSOR_VIEW) {
        adjustSelected(1);
      }
    } else {
      selectedMenu =
        (MenuItem)((selectedMenu + 1) % MENU_COUNT);
    }
  }

  if (pressed(BTN_PREV)) {
    if (editing) {
      if (selectedMenu != MENU_DIAGNOSTICS &&
          selectedMenu != MENU_SENSOR_VIEW) {
        adjustSelected(-1);
      }
    } else {
      selectedMenu =
        (MenuItem)(
          (selectedMenu + MENU_COUNT - 1) %
          MENU_COUNT
        );
    }
  }
}

// -----------------------------------------------------------------------------
// PID controller
// -----------------------------------------------------------------------------

static int16_t calculatePidCorrection(
  float error,
  float dt
) {
  dt = constrain(
    dt,
    MIN_DT_S,
    MAX_DT_S
  );

  const float derivativeRaw =
    (error - lastError) / dt;

  derivativeFiltered =
    derivativeFiltered +
    DERIVATIVE_ALPHA *
    (derivativeRaw - derivativeFiltered);

  // Candidate integral.
  const float candidateIntegral =
    integral + error * dt;

  const float candidateCorrection =
    kp * error +
    ki * candidateIntegral +
    kd * derivativeFiltered;

  // Conditional integration:
  // if the output would saturate and the error pushes farther into
  // saturation, do not integrate further in that direction.
  const float outputLimit =
    (float)maxSpeed;

  const bool positiveSaturated =
    candidateCorrection > outputLimit &&
    error > 0.0f;

  const bool negativeSaturated =
    candidateCorrection < -outputLimit &&
    error < 0.0f;

  if (!positiveSaturated &&
      !negativeSaturated) {
    integral = candidateIntegral;
  }

  const float correction =
    kp * error +
    ki * integral +
    kd * derivativeFiltered;

  return constrain(
    (int16_t)correction,
    -maxSpeed,
    maxSpeed
  );
}

// -----------------------------------------------------------------------------
// Line-loss / fork logic
// -----------------------------------------------------------------------------

static void updateForkPreference(
  int32_t position,
  uint32_t nowMs
) {
  if (forkPreference != 0 &&
      nowMs >= forkPreferenceUntil) {
    forkPreference = 0;
  }

  const bool centered =
    abs(position - (int32_t)(
      (SENSOR_COUNT - 1) * 1000 / 2
    )) < FORK_CENTER_TOLERANCE;

  const bool leftMark =
    sensorNorm[0] >= FORK_MARK_THRESHOLD ||
    sensorNorm[1] >= FORK_MARK_THRESHOLD;

  const bool rightMark =
    sensorNorm[SENSOR_COUNT - 1] >= FORK_MARK_THRESHOLD ||
    sensorNorm[SENSOR_COUNT - 2] >= FORK_MARK_THRESHOLD;

  if (centered && leftMark != rightMark) {
    forkPreference =
      leftMark ? -1 : 1;

    forkPreferenceUntil =
      nowMs + FORK_MARK_WINDOW_MS;
  }
}

static int16_t calculateLostLineTurn() {
  const int32_t center =
    (SENSOR_COUNT - 1) * 1000 / 2;

  const int32_t distance =
    abs(lastPosition - center);

  const int16_t power =
    constrain(
      LOST_LINE_POWER_MIN +
      (int16_t)(distance / 15),
      LOST_LINE_POWER_MIN,
      LOST_LINE_POWER_MAX
    );

  if (lastPosition < center) {
    return -power;
  }

  return power;
}

// -----------------------------------------------------------------------------
// Control frame
// -----------------------------------------------------------------------------

static void runControlFrame() {
  if (mode == MODE_MAZE_RUNNING) {
    runMazeFrame();
    return;
  }

  const uint32_t frameStart = micros();

  const uint32_t nowUs = frameStart;

  if (lastFrameAt != 0) {
    lastFramePeriodUs =
      nowUs - lastFrameAt;

    if (lastFramePeriodUs > 0) {
      measuredHz =
        1000000.0f /
        (float)lastFramePeriodUs;
    }

    if (lastFramePeriodUs < minFramePeriodUs) {
      minFramePeriodUs = lastFramePeriodUs;
    }

    if (lastFramePeriodUs > maxFramePeriodUs) {
      maxFramePeriodUs = lastFramePeriodUs;
    }

    if (lastFramePeriodUs >
        CONTROL_PERIOD_US * 2) {
      controlOverrunCount++;
      controlOverrunAt = millis();
    }
  }

  lastFrameAt = nowUs;
  frameCount++;

  const uint32_t scanUs = readSensors();

  if (scanUs >= CONTROL_PERIOD_US) {
    // The sensor frame itself is consuming the complete target period.
    // This is diagnostic information, not an automatic fault.
    controlOverrunCount++;
    controlOverrunAt = millis();
  }

  normalizeSensors(activeBlackLine);

  uint16_t lineStrength = 0;
  uint16_t peakStrength = 0;
  uint8_t activeCount = 0;
  float confidence = 0.0f;

  int32_t position =
    calculatePosition(
      &lineStrength,
      &peakStrength,
      &activeCount,
      &confidence
    );

  lastLineStrength = lineStrength;
  lastPeakStrength = peakStrength;
  activeSensorCount = activeCount;
  lastConfidence = confidence;

  const uint32_t nowMs = millis();

  const bool inversionCandidate =
    isColorInversionCandidate(lineStrength);

  if (updateLinePolarity(&lineStrength)) {
    uint32_t weightedSum = 0;
    uint32_t sum = 0;

    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
      weightedSum +=
        (uint32_t)sensorNorm[i] *
        (uint32_t)(i * 1000);

      sum += sensorNorm[i];
    }

    if (sum > 0) {
      position =
        weightedSum / sum;

      lastPosition = position;
    }

    // Recompute peak/count/confidence after polarity changes so the
    // line-present decision uses the new sensor polarity.
    updateLineMetrics(
      &lineStrength,
      &peakStrength,
      &activeCount,
      &confidence
    );

    lastLineStrength = lineStrength;
    lastPeakStrength = peakStrength;
    activeSensorCount = activeCount;
    lastConfidence = confidence;
  }

  const bool linePresent =
    peakStrength >= LINE_MIN_PEAK &&
    (
      lineStrength >= LINE_LOST_THRESHOLD ||
      (
        lineStrength >= LINE_WIDE_THRESHOLD &&
        !inversionCandidate
      )
    ) &&
    confidence >= 0.15f;

  const bool sideBranchPresent =
    detectSideBranch();

  if (linePresent) {
    lastLineSeenAt = nowMs;

    if (abs(
          position -
          (int32_t)((SENSOR_COUNT - 1) * 1000 / 2)
        ) <= DEAD_END_CENTER_TOLERANCE &&
        lineStrength >= DEAD_END_MIN_STRENGTH) {

      centeredStrongLineAt = nowMs;
    }

    if (sideBranchPresent) {
      lastSideBranchAt = nowMs;
    }

    // A confirmed line frame cancels a pending dead-end candidate.
    deadEndCandidate = false;
    deadEndCandidateAt = 0;

    updateForkPreference(
      position,
      nowMs
    );
  }

  // Real frame-to-frame dt.
  static uint32_t previousFrameUs = 0;

  float dt = CONTROL_PERIOD_US * 1e-6f;

  if (previousFrameUs != 0) {
    uint32_t actualDtUs =
      nowUs - previousFrameUs;

    dt =
      constrain(
        actualDtUs * 1e-6f,
        MIN_DT_S,
        MAX_DT_S
      );
  }

  previousFrameUs = nowUs;

  // Smooth start: reduce base speed but continue using PID steering.
  float effectiveBase = baseSpeed;

  const uint32_t sinceStart =
    nowMs - lastStartPressedAt;

  if (sinceStart < START_RAMP_MS) {
    const float progress =
      (float)sinceStart /
      (float)START_RAMP_MS;

    effectiveBase =
      START_MIN_POWER +
      (baseSpeed - START_MIN_POWER) *
      progress;
  }

  // ---------------------------------------------------------------------------
  // Dead-end / U-turn maneuver
  // ---------------------------------------------------------------------------

  if (serviceUTurn(
        linePresent,
        confidence,
        nowMs
      )) {

    lastControlUs =
      micros() - frameStart;

    if (lastControlUs > maxControlUs) {
      maxControlUs = lastControlUs;
    }

    return;
  }

  // ---------------------------------------------------------------------------
  // Line lost / dead-end detection
  // ---------------------------------------------------------------------------

  if (!linePresent) {

    if (nowMs - lastLineSeenAt <=
        LINE_GAP_HOLD_MS) {

      // Preserve the last steering command only for a short physical gap.
      setMotors(
        lastDriveLeft,
        lastDriveRight
      );

      lastControlUs =
        micros() - frameStart;

      if (lastControlUs > maxControlUs) {
        maxControlUs = lastControlUs;
      }

      return;
    }

    // A dead end is a centered, strong line that suddenly terminates with
    // no recent left/right branch. Confirm the absence before turning.
    if (!deadEndCandidate &&
        deadEndConditionsReady(
          nowMs
        ) &&
        !sideBranchPresent) {

      deadEndCandidate = true;
      deadEndCandidateAt = nowMs;

      Serial.println(
        F("DEAD END: candidate.")
      );
    }

    if (deadEndCandidate &&
        nowMs - deadEndCandidateAt >=
          DEAD_END_CONFIRM_MS) {

      // Turn toward the side where the line was last biased. If centered,
      // default to the right.
      const int32_t center =
        (SENSOR_COUNT - 1) * 1000 / 2;

      const int8_t direction =
        lastPosition < center ? -1 : 1;

      beginUTurn(direction);

      lastControlUs =
        micros() - frameStart;

      if (lastControlUs > maxControlUs) {
        maxControlUs = lastControlUs;
      }

      return;
    }

    if (nowMs - lastLineSeenAt >=
        LOST_LINE_SEARCH_MS) {

      motorStop();

      Serial.println(
        F("LINE LOST: safety stop.")
      );

      mode = MODE_FAULT;

      return;
    }

    const int16_t turn =
      calculateLostLineTurn();

    setMotors(
      -turn,
      turn
    );

    lastDriveLeft = -turn;
    lastDriveRight = turn;

    return;
  }

  // ---------------------------------------------------------------------------
  // PID
  // ---------------------------------------------------------------------------

  const int32_t center =
    (SENSOR_COUNT - 1) * 1000 / 2;

  const float error =
    (float)position -
    (float)center;

  int16_t correction =
    calculatePidCorrection(
      error,
      dt
    );

  if (lineStrength >= LINE_WIDE_THRESHOLD &&
      forkPreference != 0) {

    correction +=
      forkPreference *
      FORK_TURN_BIAS;

    correction =
      constrain(
        correction,
        -maxSpeed,
        maxSpeed
      );
  }

  int16_t left =
    (int16_t)(effectiveBase + correction);

  int16_t right =
    (int16_t)(effectiveBase - correction);

  // Differential output saturation.
  const int16_t maxMagnitude =
    max(
      abs(left),
      abs(right)
    );

  if (maxMagnitude > maxSpeed) {
    const float scale =
      (float)maxSpeed /
      (float)maxMagnitude;

    left =
      (int16_t)(left * scale);

    right =
      (int16_t)(right * scale);
  }

  left = clampSpeed(left);
  right = clampSpeed(right);

  setMotors(left, right);

  lastDriveLeft = left;
  lastDriveRight = right;

  lastError = error;

  lastControlUs =
    micros() - frameStart;

  if (lastControlUs > maxControlUs) {
    maxControlUs = lastControlUs;
  }
}

// -----------------------------------------------------------------------------
// OLED helpers
// -----------------------------------------------------------------------------


static const uint8_t devtaLogoBitmap[] PROGMEM = {
  0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x20,0x00,0x00, 0x00,0x00,0x00,0x60,0x00,0x00,
  0x00,0x00,0x00,0x60,0x00,0x00, 0x00,0x00,0x00,0x60,0x00,0x00,
  0x00,0x00,0x0C,0x61,0x00,0x00, 0x00,0x00,0x04,0x73,0x00,0x00,
  0x00,0x00,0x06,0xF3,0x00,0x00, 0x00,0x00,0x06,0xF3,0x00,0x00,
  0x00,0x00,0x06,0xF3,0x00,0x00, 0x00,0x00,0x0C,0xF3,0x00,0x00,
  0x00,0x00,0x0C,0x73,0x80,0x00, 0x00,0x00,0x0C,0x61,0x80,0x00,
  0x00,0x00,0x1C,0x71,0x80,0x00, 0x00,0x00,0x1F,0x67,0x80,0x00,
  0x00,0x00,0x0F,0xFF,0x80,0x00, 0x00,0x00,0x0F,0xFF,0x00,0x00,
  0x00,0x00,0x03,0xFE,0x00,0x00, 0x00,0x00,0x00,0xF0,0x00,0x00,
  0x00,0x00,0x00,0x60,0x00,0x00, 0x00,0x00,0x06,0x64,0x00,0x00,
  0x00,0x01,0xF3,0x6C,0x00,0x00, 0x00,0x03,0xB9,0xF8,0x00,0x00,
  0x00,0x02,0x0C,0xE0,0x00,0x00, 0x00,0x04,0x0C,0x78,0x00,0x00,
  0x03,0x84,0x0D,0xFC,0x00,0x00, 0x03,0xC0,0x0D,0x2E,0x00,0x00,
  0x04,0x60,0x0B,0xFF,0x00,0x00, 0x02,0x60,0x13,0xFB,0x00,0x00,
  0x00,0x38,0x1D,0xFB,0x00,0x00, 0x00,0x30,0x06,0x23,0x00,0x00,
  0x00,0x38,0x06,0x23,0x00,0x00, 0x00,0x1C,0x03,0x07,0x00,0x00,
  0x00,0x18,0x03,0x06,0x00,0x00, 0x00,0x1C,0x03,0x0E,0x80,0x00,
  0x00,0x0C,0x03,0x1D,0x00,0x00, 0x00,0x0E,0x07,0x39,0x00,0x00,
  0x00,0x07,0x0E,0x72,0x00,0x00, 0x00,0x03,0xFC,0xE4,0x00,0x00,
  0x00,0x01,0xF9,0xC8,0x00,0x00, 0x00,0x00,0x01,0x90,0x00,0x00,
  0x00,0x00,0x03,0x20,0x00,0x00, 0x00,0x00,0x03,0x40,0x00,0x00,
  0x00,0x00,0x06,0x00,0x00,0x00, 0x00,0x00,0x06,0x00,0x00,0x00,
  0x00,0x00,0x06,0x00,0x00,0x00, 0x00,0x00,0x06,0x00,0x00,0x00,
  0x00,0x00,0x06,0x00,0x20,0x00, 0x00,0x00,0x06,0x00,0x20,0x00,
  0x00,0x00,0x03,0x00,0x40,0x00, 0x00,0x00,0x03,0x00,0xC0,0x00,
  0x00,0x00,0x01,0xC1,0x80,0x00, 0x00,0x00,0x00,0x7F,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00
};

static void showBootLogo() {
  if (!displayReady) {
    return;
  }

  display.clearDisplay();
  display.drawBitmap(
    (128 - 48) / 2,
    0,    devtaLogoBitmap,
    48,
    64,
    SSD1306_WHITE
  );
  display.display();

  // Boot splash only; normal UI remains non-blocking after this.
  delay(1800);
}

static void drawCenteredText(
  const char *text,
  int16_t y,
  uint8_t textSize
) {
  int16_t x1;
  int16_t y1;
  uint16_t w;
  uint16_t h;

  display.setTextSize(textSize);

  display.getTextBounds(
    text,
    0,
    y,
    &x1,
    &y1,
    &w,
    &h
  );

  display.setCursor(
    (128 - w) / 2,
    y
  );

  display.print(text);
}

static void drawMenu() {
  MenuItem previous =
    menuOffset(selectedMenu, -1);

  MenuItem next =
    menuOffset(selectedMenu, 1);

  drawCenteredText(
    menuName(previous),
    1,
    1
  );

  display.drawFastHLine(
    0,
    14,
    128,
    SSD1306_WHITE
  );
  drawCenteredText(
    menuName(selectedMenu),
    20,
    2
  );

  drawCenteredText(
    menuValueText(selectedMenu).c_str(),
    40,
    1
  );

  display.drawFastHLine(
    0,
    49,
    128,
    SSD1306_WHITE
  );

  drawCenteredText(
    menuName(next),
    55,
    1
  );
}

static void drawDiagnostics() {
  display.setTextSize(1);

  display.setCursor(0, 0);
  display.print("V2 DIAG");

  display.setCursor(72, 0);
  display.print("Hz ");
  display.print(measuredHz, 0);

  display.setCursor(0, 12);
  display.print("Scan ");
  display.print(lastSensorScanUs);
  display.print("/");
  display.print(maxSensorScanUs);

  display.setCursor(0, 22);
  display.print("Frame ");
  display.print(lastFramePeriodUs);

  display.setCursor(0, 32);
  display.print("Ctrl ");
  display.print(lastControlUs);
  display.print("/");
  display.print(maxControlUs);

  display.setCursor(0, 42);
  display.print("OVR ");
  display.print(controlOverrunCount);

  display.setCursor(64, 42);
  display.print("Conf ");
  display.print(lastConfidence, 2);

  display.setCursor(0, 54);
  display.print("Peak ");
  display.print(lastPeakStrength);

  display.setCursor(70, 54);
  display.print("S ");
  display.print(activeSensorCount);

  display.setCursor(0, 63);
  display.print(
    maneuverState == MANEUVER_UTURN
      ? "UTURN "
      : "NORMAL "
  );
  display.print("DE ");
  display.print(deadEndCount);
}

static void drawSensorView() {
  display.setTextSize(1);

  display.setCursor(0, 0);
  display.print("SENSORS");

  int16_t markerX =
    map(
      lastPosition,
      0,
      (SENSOR_COUNT - 1) * 1000,
      2,
      125
    );

  markerX =
    constrain(
      markerX,
      2,
      125
    );

  display.fillTriangle(
    markerX,
    10,
    markerX - 3,
    15,
    markerX + 3,
    15,
    SSD1306_WHITE
  );

  display.drawFastHLine(
    0,
    18,
    128,
    SSD1306_WHITE
  );

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    const int16_t x =
      i * 8 + 1;

    const uint8_t height =
      map(
        sensorNorm[i],
        0,
        1000,
        0,
        30
      );

    display.drawRect(
      x,
      22,
      6,
      30,
      SSD1306_WHITE
    );

    if (height > 0) {
      display.fillRect(
        x + 1,
        52 - height,
        4,
        height,
        SSD1306_WHITE
      );
    }
  }

  display.setCursor(0, 56);
  display.print("P ");
  display.print(lastPosition);

  display.setCursor(82, 56);
  display.print("B ");
  display.print(lastConfidence, 1);
}

static void updateDisplay() {
  if (!displayReady) {
    return;
  }

  const uint32_t now = millis();

  if (now - lastUiAt < UI_PERIOD_MS) {
    return;
  }

  lastUiAt = now;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  if (mode == MODE_RUNNING ||
      mode == MODE_MAZE_RUNNING) {

    display.setTextSize(1);

    display.setCursor(0, 0);

    if (mode == MODE_MAZE_RUNNING) {
      display.print("MAZE ");
      display.print(measuredHz, 0);
      display.print("Hz");
    } else {
      display.print("RUN V2 ");
      display.print(measuredHz, 0);
      display.print("Hz");
    }

    display.setCursor(0, 12);
    display.print("Base ");
    display.print(baseSpeed);

    display.print(" Max ");
    display.print(maxSpeed);

    display.setCursor(0, 22);
    display.print("Pos ");
    display.print(lastPosition);

    display.print(" Err ");
    display.print(lastError, 0);

    display.setCursor(0, 32);
    display.print("Scan ");
    display.print(lastSensorScanUs);
    display.print("us");

    display.setCursor(0, 42);
    display.print("Conf ");
    display.print(lastConfidence, 2);

    display.print(" O ");
    display.print(controlOverrunCount);

    display.setCursor(0, 54);
    display.print(
      maneuverState == MANEUVER_UTURN
        ? "U-TURN"
        : activeBlackLine ? "BLACK " : "WHITE "
    );

    if (mode == MODE_MAZE_RUNNING) {
      display.print(" ");
      display.print(
        mazeState == MAZE_TURNING ? "TURN" :
        mazeState == MAZE_PROBE ? "PROBE" :
        mazeState == MAZE_FINISHED ? "DONE" :
        "FOLLOW"
      );

      display.setCursor(0, 63);
      display.print("J");
      display.print(mazeJunctionCount);
      display.print(" D");
      display.print(mazeDeadEndCount);
      display.print(" P");
      display.print(mazePathLength);
    } else if (maneuverState != MANEUVER_UTURN) {
      display.print(
        forkPreference < 0
          ? "FORK L"
          : forkPreference > 0
            ? "FORK R"
            : "FORK -"
      );
    }

  } else if (mode == MODE_CALIBRATING) {

    display.setTextSize(1);

    display.setCursor(0, 0);
    display.print("CALIBRATING V2");

    uint32_t elapsed =
      millis() - calibrationStartedAt;

    display.setCursor(0, 14);
    display.print("Time ");

    display.print(
      min(
        elapsed,
        (uint32_t)CALIBRATION_TIME_MS
      ) / 1000.0f,
      1
    );

    display.print("/");
    display.print(
      CALIBRATION_TIME_MS / 1000
    );

    display.print("s");

    display.setCursor(0, 28);

    if (calibrationLooksValid()) {
      display.print("Range: OK");
    } else {
      display.print("Move across line/bg");
    }

    display.setCursor(0, 44);
    display.print("Do not hold still");

  } else if (mode == MODE_FAULT) {

    display.setTextSize(2);
    drawCenteredText(
      "FAULT",
      5,
      2
    );

    display.setTextSize(1);
    display.setCursor(0, 30);

    if (!calibrationLooksValid()) {
      display.print("Calibration invalid");
    } else if (mazeFault) {
      display.print("Maze/safety stop");
    } else {
      display.print("Line/safety stop");
    }

    display.setCursor(0, 48);
    display.print("START = acknowledge");

  } else {

    if (editing &&
        selectedMenu == MENU_DIAGNOSTICS) {

      drawDiagnostics();

    } else if (
      editing &&
      selectedMenu == MENU_SENSOR_VIEW
    ) {

      drawSensorView();

    } else if (editing) {

      display.setTextSize(1);

      display.setCursor(0, 0);
      display.print("EDIT");

      display.setCursor(0, 16);
      display.print(
        menuName(selectedMenu)
      );

      display.setTextSize(2);

      display.setCursor(0, 32);
      display.print(
        menuValueText(selectedMenu)
      );

    } else {

      drawMenu();
    }
  }

  display.display();
}

// -----------------------------------------------------------------------------
// Telemetry
// -----------------------------------------------------------------------------

static void updateTelemetry() {
  const uint32_t now = millis();

  if (now - lastTelemetryAt <
      TELEMETRY_PERIOD_MS) {
    return;
  }

  lastTelemetryAt = now;

  Serial.print(F("V2 "));
  Serial.print(F("Hz="));
  Serial.print(measuredHz, 1);

  Serial.print(F(" scan="));
  Serial.print(lastSensorScanUs);

  Serial.print(F("us frame="));
  Serial.print(lastFramePeriodUs);

  Serial.print(F("us ctrl="));
  Serial.print(lastControlUs);

  Serial.print(F("us ovr="));
  Serial.print(controlOverrunCount);

  Serial.print(F(" conf="));
  Serial.print(lastConfidence, 2);

  Serial.print(F(" pos="));
  Serial.print(lastPosition);

  Serial.print(F(" strength="));
  Serial.print(lastLineStrength);

  Serial.print(F(" peak="));
  Serial.print(lastPeakStrength);

  Serial.print(F(" deadEnds="));
  Serial.print(deadEndCount);

  Serial.print(F(" maneuver="));
  Serial.print(
    maneuverState == MANEUVER_UTURN
      ? F("UTURN")
      : F("NORMAL")
  );

  Serial.print(F(" mode="));
  Serial.print(
    mode == MODE_MAZE_RUNNING ? F("MAZE") : F("LINE")
  );

  if (mode == MODE_MAZE_RUNNING ||
      mazePathLength > 0) {
    Serial.print(F(" mazeJ="));
    Serial.print(mazeJunctionCount);
    Serial.print(F(" mazeD="));
    Serial.print(mazeDeadEndCount);
    Serial.print(F(" mazeP="));
    Serial.print(mazePathLength);
    Serial.print(F(" path="));
    Serial.print(mazePath);
  }

  Serial.println();
}

// -----------------------------------------------------------------------------
// Control scheduler
// -----------------------------------------------------------------------------

static void serviceControl() {
  static uint32_t nextControlAt = 0;

  const uint32_t now = micros();

  if (nextControlAt == 0) {
    nextControlAt = now;
  }

  if ((int32_t)(now - nextControlAt) < 0) {
    return;
  }

  // Run exactly one frame. Do not execute catch-up frames in a burst.
  runControlFrame();

  const uint32_t after = micros();

  if ((int32_t)(after - nextControlAt) >
      (int32_t)(CONTROL_PERIOD_US * 2)) {

    controlOverrunCount++;
    controlOverrunAt = millis();

    // Drop missed deadlines instead of executing a backlog.
    nextControlAt =
      after + CONTROL_PERIOD_US;

  } else {

    nextControlAt +=
      CONTROL_PERIOD_US;
  }
}

// -----------------------------------------------------------------------------
// Arduino lifecycle
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);

  btStop();

  setupSensors();
  loadSettings();

  if (!setupMotors()) {
    mode = MODE_FAULT;

    Serial.println(
      F("FAULT: LEDC motor setup failed.")
    );
  }

  setupButtons();

  motorStop();

  Wire.begin(
    PIN_I2C_SDA,
    PIN_I2C_SCL
  );

  displayReady =
    display.begin(
      SSD1306_SWITCHCAPVCC,
      0x3C
    );

  if (displayReady) {
    display.setTextColor(SSD1306_WHITE);
    showBootLogo();

    // Start normal UI timing after the splash.
    lastUiAt = millis();
  }

  Serial.println();
  Serial.println(
    F("====================================")
  );
  Serial.println(
    F("DEVTA COMPETITION LINE FOLLOWER V2")
  );
  Serial.println(
    F("Target control period: 2000 us")
  );
  Serial.println(
    F("Target frequency: 500 Hz")
  );
  Serial.println(
    F("Actual frequency will be measured.")
  );
  Serial.println(
    F("Drive mode: menu -> Drive mode -> Line/Maze")
  );
  Serial.println(
    F("Maze mode: left-hand exploration + LSRB path reduction.")
  );
  Serial.println(
    F("====================================")
  );

  if (!calibrationLooksValid()) {
    Serial.println(
      F("Calibration not valid. Calibrate before RUN.")
    );
  }
}

void loop() {
  handleButtons();

  if (mode == MODE_CALIBRATING) {

    updateCalibration();

    if (millis() - calibrationStartedAt >=
        CALIBRATION_TIME_MS) {

      finishCalibration();
    }

  } else if (mode == MODE_RUNNING ||
             mode == MODE_MAZE_RUNNING) {

    serviceControl();

  } else if (mode == MODE_IDLE) {

    // No repeated motor writes while idle.
    digitalWrite(PIN_MOTOR_STBY, LOW);

  } else if (mode == MODE_FAULT) {

    motorStop();
  }

  updateDisplay();
  updateTelemetry();
}