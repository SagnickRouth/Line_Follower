/*
  Fast ESP32 line follower firmware

  Board/module: ESP32-WROOM based board
  Driver: TB6612FNG
  Sensors: 16-channel analog IR array with onboard multiplexer
  UI: SSD1306 OLED + 6 buttons

  Libraries:
    - Adafruit SSD1306
    - Adafruit GFX Library

  Notes:
    - MUX_OUT is on an ADC1 pin so sensor reads stay stable.
    - OLED/UI is deliberately updated slowly so the PID loop stays fast.
*/

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------------------------- Pin map ----------------------------
// Change these to match your wiring.

// 16-channel mux IR array:
//   MUX_OUT -> GPIO36
//   S0      -> GPIO32
//   S1      -> GPIO33
//   S2      -> GPIO25
//   S3      -> GPIO26
//   VIN x2  -> sensor-rated VIN
//   GND x3  -> common GND
static const uint8_t SENSOR_COUNT = 16;
static const uint8_t PIN_SENSOR_MUX_OUT = 36;
static const uint8_t PIN_SENSOR_S0 = 32;
static const uint8_t PIN_SENSOR_S1 = 33;
static const uint8_t PIN_SENSOR_S2 = 25;
static const uint8_t PIN_SENSOR_S3 = 26;

// TB6612FNG motor driver:
//   Left motor  -> A output
//   Right motor -> B output
static const uint8_t PIN_MOTOR_L_PWM = 23;
static const uint8_t PIN_MOTOR_L_IN1 = 18;
static const uint8_t PIN_MOTOR_L_IN2 = 19;
static const uint8_t PIN_MOTOR_R_PWM = 5;
static const uint8_t PIN_MOTOR_R_IN1 = 16;
static const uint8_t PIN_MOTOR_R_IN2 = 17;
static const uint8_t PIN_MOTOR_STBY = 4;

// Buttons are active-low: connect each button between GPIO and GND.
// GPIO34 and GPIO35 need external pull-ups because they have no internal pull-up.
static const uint8_t PIN_BTN_NEXT = 13;
static const uint8_t PIN_BTN_PREV = 14;
static const uint8_t PIN_BTN_ENTER = 27;
static const uint8_t PIN_BTN_BACK = 12;
static const uint8_t PIN_BTN_CAL = 35;
static const uint8_t PIN_BTN_START = 34;

// SSD1306 OLED I2C bus.
static const uint8_t PIN_I2C_SDA = 21;
static const uint8_t PIN_I2C_SCL = 22;

// ---------------------------- Control settings ----------------------------

static const uint16_t ADC_MAX_VALUE = 4095;
static const uint16_t CONTROL_PERIOD_US = 1000;   // 1 kHz target loop
static const uint16_t UI_PERIOD_MS = 100;
static const uint16_t BUTTON_PERIOD_MS = 10;
static const uint16_t LINE_LOST_THRESHOLD = SENSOR_COUNT * 10;
static const uint16_t LOST_LINE_POWER = 80;
static const uint16_t CALIBRATION_TIME_MS = 5000;
static const uint16_t SPLASH_NAME_MS = 1400;
static const uint16_t SPLASH_TRIDENT_MS = 1500;
static const uint16_t START_MESSAGE_MS = 1200;
static const uint16_t START_KICK_MS = 300;
static const uint16_t LINE_CENTER = ((SENSOR_COUNT - 1) * 1000) / 2;
static const uint8_t MUX_SETTLE_US = 3;

static const uint8_t PWM_BITS = 10;
static const uint16_t PWM_MAX = (1 << PWM_BITS) - 1;
static const uint32_t PWM_FREQ = 20000;
static const uint8_t PWM_CH_L = 0;
static const uint8_t PWM_CH_R = 1;

// ---------------------------- Speed settings ----------------------------

// ONLY CHANGE:
// Four selectable speed values.
static const int16_t SPEED_VALUES[] = {
  100,
  360,
  840,
  1038
};

static const uint8_t SPEED_COUNT = 4;

// Initial values remain valid selectable values.
int16_t baseSpeed = 360;
int16_t maxSpeed = 840;

// Start conservative. Raise base speed after the robot tracks reliably.
float kp = 0.095f;
float ki = 0.000f;
float kd = 0.62f;

// Set true if your track is a black line on a white background.
bool blackLine = true;

// If one motor is physically reversed, flip it here.
bool invertLeftMotor = false;
bool invertRightMotor = false;

// ---------------------------- UI state ----------------------------

enum RunMode {
  MODE_IDLE,
  MODE_RUNNING,
  MODE_CALIBRATING
};

enum MenuItem {
  MENU_BASE_SPEED,
  MENU_MAX_SPEED,
  MENU_KP,
  MENU_KI,
  MENU_KD,
  MENU_LINE_COLOR,
  MENU_SENSOR_VIEW,
  MENU_COUNT
};

struct Button {
  uint8_t pin;
  bool stable;
  bool previousStable;
  bool rawLast;
  uint32_t changedAt;
  bool pressedEvent;
};

Button buttons[] = {
  {PIN_BTN_NEXT, true, true, true, 0, false},
  {PIN_BTN_PREV, true, true, true, 0, false},
  {PIN_BTN_ENTER, true, true, true, 0, false},
  {PIN_BTN_BACK, true, true, true, 0, false},
  {PIN_BTN_CAL, true, true, true, 0, false},
  {PIN_BTN_START, true, true, true, 0, false},
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

// ---------------------------- Sensor/control state ----------------------------

uint16_t sensorRaw[SENSOR_COUNT];
uint16_t sensorNorm[SENSOR_COUNT];
uint16_t sensorMin[SENSOR_COUNT];
uint16_t sensorMax[SENSOR_COUNT];

int32_t lastPosition = LINE_CENTER;
float integral = 0.0f;
float lastError = 0.0f;
uint32_t lastControlAt = 0;
uint32_t lastUiAt = 0;
uint32_t lastButtonAt = 0;
uint32_t calibrationStartedAt = 0;
uint32_t controlLoopCount = 0;
uint32_t lastStartPressedAt = 0;
uint32_t lastStopPressedAt = 0;
uint32_t startKickUntil = 0;

Preferences prefs;
Adafruit_SSD1306 display(128, 64, &Wire, -1);
bool displayReady = false;

// ---------------------------- Utility ----------------------------

static int16_t clampSpeed(int16_t value) {
  if (value > maxSpeed) return maxSpeed;
  if (value < -maxSpeed) return -maxSpeed;
  return value;
}

static void saveSettings() {
  prefs.begin("lf-robot", false);
  prefs.putFloat("kp", kp);
  prefs.putFloat("ki", ki);
  prefs.putFloat("kd", kd);
  prefs.putShort("base", baseSpeed);
  prefs.putShort("max", maxSpeed);
  prefs.putBool("black", blackLine);

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    char keyMin[8];
    char keyMax[8];
    snprintf(keyMin, sizeof(keyMin), "mn%u", i);
    snprintf(keyMax, sizeof(keyMax), "mx%u", i);
    prefs.putUShort(keyMin, sensorMin[i]);
    prefs.putUShort(keyMax, sensorMax[i]);
  }

  prefs.end();
}

static void loadSettings() {
  prefs.begin("lf-robot", true);
  kp = prefs.getFloat("kp", kp);
  ki = prefs.getFloat("ki", ki);
  kd = prefs.getFloat("kd", kd);
  baseSpeed = prefs.getShort("base", baseSpeed);
  maxSpeed = prefs.getShort("max", maxSpeed);
  blackLine = prefs.getBool("black", blackLine);

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    char keyMin[8];
    char keyMax[8];
    snprintf(keyMin, sizeof(keyMin), "mn%u", i);
    snprintf(keyMax, sizeof(keyMax), "mx%u", i);
    sensorMin[i] = prefs.getUShort(keyMin, 300);
    sensorMax[i] = prefs.getUShort(keyMax, 3800);

    if (sensorMax[i] <= sensorMin[i] + 10) {
      sensorMin[i] = 300;
      sensorMax[i] = 3800;
    }
  }

  prefs.end();
}

static void resetCalibrationBounds() {
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    sensorMin[i] = ADC_MAX_VALUE;
    sensorMax[i] = 0;
  }
}

// ---------------------------- Motor driver ----------------------------

static void setupMotors() {
  pinMode(PIN_MOTOR_L_IN1, OUTPUT);
  pinMode(PIN_MOTOR_L_IN2, OUTPUT);
  pinMode(PIN_MOTOR_R_IN1, OUTPUT);
  pinMode(PIN_MOTOR_R_IN2, OUTPUT);
  pinMode(PIN_MOTOR_STBY, OUTPUT);

  ledcSetup(PWM_CH_L, PWM_FREQ, PWM_BITS);
  ledcSetup(PWM_CH_R, PWM_FREQ, PWM_BITS);
  ledcAttachPin(PIN_MOTOR_L_PWM, PWM_CH_L);
  ledcAttachPin(PIN_MOTOR_R_PWM, PWM_CH_R);

  digitalWrite(PIN_MOTOR_STBY, HIGH);
}

static void setOneMotor(uint8_t pwmChannel, uint8_t in1, uint8_t in2, int16_t speed, bool invert) {
  if (invert) speed = -speed;
  speed = constrain(speed, -(int16_t)PWM_MAX, (int16_t)PWM_MAX);

  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    ledcWrite(pwmChannel, speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    ledcWrite(pwmChannel, -speed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(pwmChannel, 0);
  }
}

static void setMotors(int16_t left, int16_t right) {
  left = clampSpeed(left);
  right = clampSpeed(right);

  setOneMotor(
    PWM_CH_L,
    PIN_MOTOR_L_IN1,
    PIN_MOTOR_L_IN2,
    left,
    invertLeftMotor
  );

  setOneMotor(
    PWM_CH_R,
    PIN_MOTOR_R_IN1,
    PIN_MOTOR_R_IN2,
    right,
    invertRightMotor
  );
}

static void stopMotors() {
  setMotors(0, 0);
  integral = 0.0f;
}

// ---------------------------- Sensors ----------------------------

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

static void readSensors() {
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    selectSensorChannel(i);
    sensorRaw[i] = analogRead(PIN_SENSOR_MUX_OUT);
  }
}

static void normalizeSensors() {
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    uint16_t lo = sensorMin[i];
    uint16_t hi = sensorMax[i];
    uint16_t raw = sensorRaw[i];
    uint16_t value = 0;

    if (hi > lo + 10) {
      raw = constrain(raw, lo, hi);
      value = map(raw, lo, hi, 0, 1000);
    }

    sensorNorm[i] = blackLine ? value : 1000 - value;
  }
}

static void updateCalibration() {
  readSensors();

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    if (sensorRaw[i] < sensorMin[i]) sensorMin[i] = sensorRaw[i];
    if (sensorRaw[i] > sensorMax[i]) sensorMax[i] = sensorRaw[i];
  }
}

static int32_t readLinePosition(uint16_t *lineStrength) {
  readSensors();
  normalizeSensors();

  uint32_t weightedSum = 0;
  uint32_t sum = 0;

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    uint16_t value = sensorNorm[i];
    weightedSum += (uint32_t)value * (i * 1000);
    sum += value;
  }

  *lineStrength = sum;

  if (sum < LINE_LOST_THRESHOLD) {
    return lastPosition;
  }

  lastPosition = weightedSum / sum;
  return lastPosition;
}

// ---------------------------- Buttons/UI ----------------------------

static void setupButtons() {
  for (Button &button : buttons) {
    if (button.pin == 34 || button.pin == 35) {
      pinMode(button.pin, INPUT);
    } else {
      pinMode(button.pin, INPUT_PULLUP);
    }

    button.stable = digitalRead(button.pin);
    button.previousStable = button.stable;
    button.rawLast = button.stable;
  }
}

static void updateButtons() {
  const uint32_t now = millis();

  if (now - lastButtonAt < BUTTON_PERIOD_MS) return;
  lastButtonAt = now;

  for (Button &button : buttons) {
    button.pressedEvent = false;

    bool raw = digitalRead(button.pin);

    if (raw != button.rawLast) {
      button.rawLast = raw;
      button.changedAt = now;
    }

    if (now - button.changedAt >= 25 && raw != button.stable) {
      button.previousStable = button.stable;
      button.stable = raw;

      if (button.previousStable == HIGH && button.stable == LOW) {
        button.pressedEvent = true;
      }
    }
  }
}

static bool pressed(uint8_t index) {
  return buttons[index].pressedEvent;
}

static void beginCalibration() {
  mode = MODE_CALIBRATING;
  calibrationStartedAt = millis();
  resetCalibrationBounds();
  stopMotors();
}

static void finishCalibration() {
  mode = MODE_IDLE;
  saveSettings();
  stopMotors();
}


// ----------------------------
// Speed adjustment
// ----------------------------

static void adjustSpeedValue(
  int16_t &speed,
  int8_t direction
) {
  int8_t currentIndex = 0;

  for (uint8_t i = 0; i < SPEED_COUNT; i++) {
    if (speed == SPEED_VALUES[i]) {
      currentIndex = i;
      break;
    }
  }

  currentIndex += direction;

  if (currentIndex < 0) {
    currentIndex = SPEED_COUNT - 1;
  }

  if (currentIndex >= SPEED_COUNT) {
    currentIndex = 0;
  }

  speed = SPEED_VALUES[currentIndex];
}


static void adjustSelected(int8_t direction) {
  switch (selectedMenu) {

    case MENU_BASE_SPEED:

      adjustSpeedValue(
        baseSpeed,
        direction
      );

      break;


    case MENU_MAX_SPEED:

      adjustSpeedValue(
        maxSpeed,
        direction
      );

      break;


    case MENU_KP:

      kp = max(
        0.0f,
        kp + direction * 0.005f
      );

      break;


    case MENU_KI:

      ki = max(
        0.0f,
        ki + direction * 0.0001f
      );

      break;


    case MENU_KD:

      kd = max(
        0.0f,
        kd + direction * 0.025f
      );

      break;


    case MENU_LINE_COLOR:

      if (direction != 0) {
        blackLine = !blackLine;
      }

      break;


    default:

      break;
  }
}

static void handleButtons() {
  updateButtons();

  if (pressed(BTN_START) && !startButtonLocked) {
    editing = false;

    startButtonLocked = true;

    if (mode == MODE_RUNNING) {
      mode = MODE_IDLE;
      lastStopPressedAt = millis();
      stopMotors();
      saveSettings();

    } else if (mode == MODE_IDLE) {
      mode = MODE_RUNNING;
      lastStartPressedAt = millis();
      startKickUntil = millis() + START_KICK_MS;
      controlLoopCount = 0;
      lastControlAt = micros();
      integral = 0.0f;
      lastError = 0.0f;
    }
  }

  if (buttons[BTN_START].stable == HIGH) {
    startButtonLocked = false;
  }

  if (pressed(BTN_CAL)) {
    beginCalibration();
  }

  if (mode != MODE_IDLE) return;

  if (pressed(BTN_ENTER)) {
    if (selectedMenu == MENU_SENSOR_VIEW) {
      editing = true;
    } else if (editing) {
      saveSettings();
    } else {
      editing = true;
    }
  }

  if (pressed(BTN_BACK)) {
    if (editing) {
      editing = false;
      saveSettings();
    }
  }

  if (pressed(BTN_NEXT)) {
    if (editing) {
      if (selectedMenu != MENU_SENSOR_VIEW) {
        adjustSelected(1);
      }
    } else {
      selectedMenu =
        (MenuItem)((selectedMenu + 1) % MENU_COUNT);
    }
  }

  if (pressed(BTN_PREV)) {
    if (editing) {
      if (selectedMenu != MENU_SENSOR_VIEW) {
        adjustSelected(-1);
      }
    } else {
      selectedMenu =
        (MenuItem)((selectedMenu + MENU_COUNT - 1) % MENU_COUNT);
    }
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

    case MENU_SENSOR_VIEW:
      return "Enter";

    default:
      return "";
  }
}

static void drawValue(MenuItem item) {
  display.print(menuValueText(item));
}

static MenuItem menuOffset(MenuItem item, int8_t offset) {
  int8_t index = (int8_t)item + offset;

  while (index < 0) index += MENU_COUNT;

  while (index >= MENU_COUNT) index -= MENU_COUNT;

  return (MenuItem)index;
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

static void drawTridentSplash() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.drawLine(64, 10, 64, 52, SSD1306_WHITE);
  display.drawLine(60, 14, 64, 8, SSD1306_WHITE);
  display.drawLine(68, 14, 64, 8, SSD1306_WHITE);

  display.drawLine(44, 16, 44, 30, SSD1306_WHITE);
  display.drawLine(44, 30, 57, 38, SSD1306_WHITE);
  display.drawLine(40, 20, 44, 14, SSD1306_WHITE);
  display.drawLine(48, 20, 44, 14, SSD1306_WHITE);

  display.drawLine(84, 16, 84, 30, SSD1306_WHITE);
  display.drawLine(84, 30, 71, 38, SSD1306_WHITE);
  display.drawLine(80, 20, 84, 14, SSD1306_WHITE);
  display.drawLine(88, 20, 84, 14, SSD1306_WHITE);

  display.drawLine(52, 52, 76, 52, SSD1306_WHITE);
  display.drawLine(56, 57, 72, 57, SSD1306_WHITE);
  display.drawLine(58, 52, 56, 57, SSD1306_WHITE);
  display.drawLine(70, 52, 72, 57, SSD1306_WHITE);

  display.display();
}

static void showStartupSplash() {
  if (!displayReady) return;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  drawCenteredText("DEVTA", 21, 3);

  display.display();

  delay(SPLASH_NAME_MS);

  drawTridentSplash();

  delay(SPLASH_TRIDENT_MS);

  lastUiAt = 0;
}

static void drawMenuCarousel() {
  MenuItem previous = menuOffset(selectedMenu, -1);
  MenuItem next = menuOffset(selectedMenu, 1);

  display.setTextColor(SSD1306_WHITE);

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

static void drawSensorView() {
  uint16_t lineStrength = 0;
  int32_t position = readLinePosition(&lineStrength);
  uint8_t strongest = 0;

  for (uint8_t i = 1; i < SENSOR_COUNT; i++) {
    if (
      sensorNorm[i] >
      sensorNorm[strongest]
    ) {
      strongest = i;
    }
  }

  display.setTextSize(1);

  display.setCursor(0, 0);
  display.print("SENSOR VIEW");

  display.setCursor(90, 0);
  display.print("S");
  display.print(strongest);

  int16_t markerX = map(
    position,
    0,
    (SENSOR_COUNT - 1) * 1000,
    2,
    125
  );

  markerX = constrain(
    markerX,
    2,
    125
  );

  display.fillTriangle(
    markerX,
    11,
    markerX - 3,
    16,
    markerX + 3,
    16,
    SSD1306_WHITE
  );

  display.drawFastHLine(
    0,
    18,
    128,
    SSD1306_WHITE
  );

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    int16_t x = i * 8 + 1;

    uint8_t barHeight = map(
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

    if (barHeight > 0) {
      display.fillRect(
        x + 1,
        52 - barHeight,
        4,
        barHeight,
        SSD1306_WHITE
      );
    }
  }

  display.setCursor(0, 56);

  if (lineStrength < LINE_LOST_THRESHOLD) {
    display.print("LOST");
  } else {
    display.print("L");
    display.print(position);
  }

  display.setCursor(82, 56);
  display.print("Back");
}

static void drawStartBanner(
  const char *line1,
  const char *line2
) {
  display.fillRect(
    0,
    0,
    128,
    18,
    SSD1306_WHITE
  );

  display.setTextColor(
    SSD1306_BLACK
  );

  display.setTextSize(1);

  display.setCursor(2, 2);
  display.print(line1);

  display.setCursor(2, 10);
  display.print(line2);

  display.setTextColor(
    SSD1306_WHITE
  );
}

static void updateDisplay() {
  if (!displayReady) return;

  const uint32_t now = millis();

  if (now - lastUiAt < UI_PERIOD_MS) return;

  lastUiAt = now;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  if (mode == MODE_RUNNING) {

    display.println("RUNNING");

    display.print("Base ");
    display.print(baseSpeed);

    display.print(" Max ");
    display.println(maxSpeed);

    display.print("Pos ");
    display.println(lastPosition);

    display.print("Loops ");
    display.println(controlLoopCount);

    if (
      now - lastStartPressedAt <
      START_MESSAGE_MS
    ) {
      drawStartBanner(
        "START PRESSED",
        "Motors enabled"
      );
    }

  } else if (mode == MODE_CALIBRATING) {

    uint32_t elapsed =
      millis() -
      calibrationStartedAt;

    display.println("CALIBRATING");

    display.print("Move over line/bg");

    display.setCursor(0, 24);

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

    display.println("s");

  } else {

    if (
      now - lastStopPressedAt <
      START_MESSAGE_MS
    ) {
      drawStartBanner(
        "STOP PRESSED",
        "Motors stopped"
      );

      display.display();

      return;
    }

    if (
      editing &&
      selectedMenu == MENU_SENSOR_VIEW
    ) {

      drawSensorView();

    } else if (editing) {

      display.println("EDIT");

      display.print(
        menuName(selectedMenu)
      );

      display.setCursor(0, 22);

      display.setTextSize(2);

      drawValue(selectedMenu);

      display.setTextSize(1);

      display.setCursor(0, 52);

      display.print(
        "Next/Prev  Back=menu"
      );

    } else {

      drawMenuCarousel();

      display.setCursor(0, 56);

      display.print("St:");

      display.print(
        digitalRead(PIN_BTN_START) == LOW
          ? "P"
          : "-"
      );

      display.print(" Cal:");

      display.print(
        digitalRead(PIN_BTN_CAL) == LOW
          ? "P"
          : "-"
      );
    }
  }

  display.display();
}

// ---------------------------- Control loop ----------------------------

static void runControlLoop() {
  const uint32_t now = micros();

  if (
    now - lastControlAt <
    CONTROL_PERIOD_US
  ) {
    return;
  }

  lastControlAt +=
    CONTROL_PERIOD_US;

  controlLoopCount++;

  uint16_t lineStrength = 0;

  int32_t position =
    readLinePosition(&lineStrength);

  float error =
    position -
    (float)LINE_CENTER;

  if (
    lineStrength <
    LINE_LOST_THRESHOLD
  ) {
    int16_t turn =
      (lastPosition < LINE_CENTER)
        ? -LOST_LINE_POWER
        : LOST_LINE_POWER;

    setMotors(
      -turn,
      turn
    );

    return;
  }

  integral += error;

  integral =
    constrain(
      integral,
      -12000.0f,
      12000.0f
    );

  float derivative =
    error - lastError;

  lastError =
    error;

  int16_t correction =
    (int16_t)(
      kp * error +
      ki * integral +
      kd * derivative
    );

  int16_t left =
    baseSpeed +
    correction;

  int16_t right =
    baseSpeed -
    correction;

  setMotors(
    left,
    right
  );
}

// ---------------------------- Arduino lifecycle ----------------------------

void setup() {
  Serial.begin(115200);
  btStop();

  setupSensors();
  loadSettings();
  setupMotors();
  setupButtons();
  stopMotors();

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
    display.clearDisplay();
    display.display();
    showStartupSplash();
  }

  lastControlAt = micros();
}

void loop() {
  handleButtons();

  if (
    mode ==
    MODE_CALIBRATING
  ) {

    updateCalibration();

    if (
      millis() -
      calibrationStartedAt >=
      CALIBRATION_TIME_MS
    ) {
      finishCalibration();
    }

  } else if (
    mode ==
    MODE_RUNNING
  ) {

    runControlLoop();

  } else {

    stopMotors();
  }

  updateDisplay();
}