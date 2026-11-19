#include <Arduino.h>
#include <M5Core2.h>
#include <SCServo.h>
#include <VL53L0X.h>
#include "config.h"
#include "ServoBus.h"
#include <esp_system.h>


namespace {
HardwareSerial servoSerial(1);
ServoBus servo;
VL53L0X tof;

int commandedSteps = -1;
int desiredSteps = -1;
int actualSteps = 0;
bool actualStepsValid = false;
bool motionEnabled = false;

int stepStartMm = 100;
int stepWidthMm = 50;

bool onStep = false;

bool motorOffVerified = false;
bool touchReady = false;
bool servoReady = false;
bool tofReady = false;
bool distanceValid = false;
uint16_t distanceMm = 0;
uint32_t lastTofSample = 0;
uint32_t nextTofRetry = 0;


int targetSteps() {
  // Internal vibration/touch diagnostics are informational only.
  onStep = motionEnabled && distanceValid && millis() - lastTofSample <= kTofStaleMs &&
           distanceMm >= stepStartMm && distanceMm < stepStartMm + stepWidthMm;
  return onStep ? kStepHeightCounts : 0;
}

void randomizeStep() {
  int next = esp_random() % 15;
  if (next >= (stepStartMm - 50) / 10) ++next;
  stepStartMm = 50 + next * 10;
}

void cycleStepWidth() {
  stepWidthMm = stepWidthMm % 50 + 10;
}

void toggleRun() {
  // Only an A-button long press changes the user's RUN/PAUSE selection.
  // Internal diagnostics do not gate the distance-based servo target.
  motionEnabled = !motionEnabled;

}

void help() {
  Serial.printf("# Actual target: +%d steps in range, 0 outside / PAUSE; noise disabled\n", kStepHeightCounts);
  Serial.println("# Click B:reapply offset, C:random position");
  Serial.println("# Hold 800ms A:run/pause C:width (1..5 cm)");
  Serial.println("# Keys b:reapply offset, w:width, [ / ]:position -/+10mm, r:random, ?:help");
}

const char* servoStatus = "Servo starting";
char servoFault[40] = {};
uint32_t nextServoRetry = 1000;
size_t baudIndex = 0;
const uint32_t baudRates[] = {
    kServoBaud, 500000, 250000, 128000, 115200, 76800, 57600, 38400};

bool readInternalRegister(uint8_t address, uint8_t reg, uint8_t& value) {
  Wire1.beginTransmission(address);
  Wire1.write(reg);
  if (Wire1.endTransmission(false) != 0) return false;
  if (Wire1.requestFrom(address, static_cast<uint8_t>(1)) != 1) return false;
  value = Wire1.read();
  return true;
}

// 読出し失敗時に0xFF等を電源ICへ書き戻さない。実際のIC IDで分岐する。
bool stopMotorChecked() {
  uint8_t id, control;
  if (!readInternalRegister(0x34, 0x03, id)) return false;
  const uint8_t reg = id == 0x03 ? 0x12 : 0x90;
  const uint8_t mask = id == 0x03 ? 0x08 : 0x80;
  if (id != 0x03 && id != 0x4A) return false;
  if (!readInternalRegister(0x34, reg, control)) return false;
  if (control & mask) {
    Wire1.beginTransmission(0x34);
    Wire1.write(reg);
    Wire1.write(static_cast<uint8_t>(control & ~mask));
    if (Wire1.endTransmission() != 0) return false;
  }
  return readInternalRegister(0x34, reg, control) && !(control & mask);
}

void checkInternalDevices() {
  motorOffVerified = stopMotorChecked();
  uint8_t mode;
  touchReady = readInternalRegister(0x38, 0xA4, mode) && mode == 0;
  Serial.printf("IO2: motor=%s touch=%s buttons=%d%d%d motion=%s\n",
                motorOffVerified ? "OFF verified" : "CHECK FAILED",
                touchReady ? "OK" : "CHECK FAILED",
                M5.BtnA.isPressed(), M5.BtnB.isPressed(), M5.BtnC.isPressed(),
                motionEnabled ? "ON" : "OFF");
}

void initializeDisplay() {
  // 1. 内部I2C（Wire1）で電源ICを確認。センサー用のWireとは別系統。
  //    電源ICが未認識の場合だけ、100ms待って最大3回再試行する。
  Wire1.setTimeOut(100);
  for (int attempt = 0;
       M5.Axp.getPmicType() == AXP::pmic_unknown && attempt < 3; ++attempt) {
    Serial.println("LCD: power IC not detected; retrying internal I2C");
    delay(100);
    M5.Axp.begin();
  }
  // 2. 検出した電源ICに合わせ、LCD本体とバックライトへ給電する。
  const auto pmic = M5.Axp.getPmicType();
  if (pmic == AXP::pmic_axp192) {
    // AXP192搭載機：LCD本体3.3V、バックライト2.8V（ライブラリ既定値）。
    M5.Axp.SetLDOVoltage(2, 3300);
    M5.Axp.SetLDOEnable(2, true);
    M5.Axp.SetLcdVoltage(2800);
    M5.Axp.SetDCDC3(true);
    Serial.println("LCD: AXP192, logic=3.3V backlight=2.8V requested");
  } else if (pmic == AXP::pmic_axp2101) {
    // AXP2101搭載機：LCD本体3.3V、バックライト3.0V（ライブラリ既定値）。
    M5.Axp.axp2101.set_lcd_and_tf_voltage(3300);
    M5.Axp.SetLcdVoltage(3000);
    Serial.println("LCD: AXP2101, logic=3.3V backlight=3.0V requested");
  } else {
    Serial.println("LCD: power IC unavailable; check internal I2C/power");
  }
  // 3. AXP192搭載機は、電源が安定してからLCDをハードウェアリセット。
  if (pmic == AXP::pmic_axp192) {
    delay(100);
    M5.Axp.axp192.SetLCDRSet(false);
    delay(100);
    M5.Axp.axp192.SetLCDRSet(true);
    delay(100);
  }
  // 4. LCDの通信・表示設定を初期化する。実際の点灯は実機で確認が必要。
  // AXP2101はここでのソフトウェアリセットを使用する。
  // 共通のSetLCDRSet()にはライブラリ0.2.0の未実装関数参照があるため使わない。
  // 電源ICが未認識でもSPIは初期化し、後続の描画処理に備える。
  M5.Lcd.begin();
  Serial.println("LCD: controller initialization completed (visual check needed)");
}

bool failServo(const char* stage) {
  actualStepsValid = false;
  if (strcmp(stage, "Servo status error") == 0) {
    snprintf(servoFault, sizeof(servoFault), "Servo status 0x%02X", servo.Error);
    servoStatus = servoFault;
  } else {
    servoStatus = stage;
  }
  Serial.printf("STS3045M: %s; ID=%u baud=%lu status=0x%02X\n",
                stage, kServoId, static_cast<unsigned long>(baudRates[baudIndex]),
                servo.Error);
  return false;
}

// 書込みの確認応答（ACK）がなくても、書込み失敗とは限らない。
// 書込み応答が無効の場合も、レジスタを読み出して値が一致するか確認する。
bool verifyWrite(int ack, int expected, uint8_t address, bool word) {
  if (!servo.echoOk) return failServo("Servo echo mismatch");
  if (ack == 1 && servo.Error != 0) return failServo("Servo status error");
  delay(2);  // 読出し前に受信データを破棄するため、遅れた書込み応答の到着を待つ。
  const int actual = word ? servo.readWord(kServoId, address)
                          : servo.readByte(kServoId, address);
  if (!servo.echoOk) return failServo("Servo echo mismatch");
  if (actual < 0) return failServo("Servo READ no reply");
  if (servo.Error != 0) return failServo("Servo status error");
  if (actual != expected) return failServo("Servo verify failed");
  if (ack != 1 && servo.Level != 0) {
    servo.Level = 0;  // Core2側の設定のみ変更。サーボの不揮発メモリ設定は変更しない。
    Serial.println("STS3045M: no write ACK; using register readback verification.");
  }
  return true;
}

// Send integer position counts directly: no degree-to-count rounding.
bool setServoSteps(int steps) {
  if (steps < -kCommandLimitCounts || steps > kCommandLimitCounts)
    return failServo("Command out of range");
  const int position = kServoZeroPosition + kServoDirection * steps;
  const int ack = servo.WritePosEx(kServoId, position, kServoSpeed, kServoAcceleration);
  if (!verifyWrite(ack, position, SMS_STS_GOAL_POSITION_L, true)) return false;
  commandedSteps = steps;

  return true;
}

bool connectServo() {
  servoSerial.end();
  servoSerial.begin(baudRates[baudIndex], SERIAL_8N1, kServoRxPin, kServoTxPin);
  servo.Level = 1;
  const int pingId = servo.Ping(kServoId);
  if (!servo.echoOk) return failServo("Servo echo mismatch");
  if (pingId != kServoId) {
    failServo("Servo PING no reply");
    baudIndex = (baudIndex + 1) % (sizeof(baudRates) / sizeof(baudRates[0]));
    return false;
  }
  if (servo.Error != 0) return failServo("Servo status error");
  const int mode = servo.readByte(kServoId, SMS_STS_MODE);
  if (!servo.echoOk || servo.Error != 0 || mode != 0)
    return failServo("Need position mode 0");
  const int minimum = servo.readWord(kServoId, SMS_STS_MIN_ANGLE_LIMIT_L);
  if (!servo.echoOk || servo.Error != 0 || minimum < 0)
    return failServo("Limit READ failed");
  const int maximum = servo.readWord(kServoId, SMS_STS_MAX_ANGLE_LIMIT_L);
  if (!servo.echoOk || servo.Error != 0 || maximum < 0)
    return failServo("Limit READ failed");
  if (minimum > kServoZeroPosition - kCommandLimitCounts ||
      maximum < kServoZeroPosition + kCommandLimitCounts ||
      maximum > kServoPositionMax)
    return failServo("Check angle limits");
  // Set the intended goal before enabling torque.
  if (!setServoSteps(0)) return false;
  const int ack = servo.EnableTorque(kServoId, 1);
  if (!verifyWrite(ack, 1, SMS_STS_TORQUE_ENABLE, false)) return false;
  Serial.printf("STS3045M ready: ID=%u baud=%lu\n", kServoId,
                static_cast<unsigned long>(baudRates[baudIndex]));
  servoStatus = "Servo ready";
  desiredSteps = -1;
  return true;
}

// Check both I2C transfer status and received byte count.
bool readTof(uint8_t reg, uint8_t* data, uint8_t size) {
  Wire.beginTransmission(kTofAddress);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(kTofAddress, size) != size) return false;
  for (uint8_t i = 0; i < size; ++i) data[i] = Wire.read();
  return true;
}

void updateTof() {
  const uint32_t now = millis();
  if (!tofReady) {
    distanceValid = false;
    // Retry the sensor independently of servo communication.
    if (static_cast<int32_t>(now - nextTofRetry) < 0) return;
    tof.setTimeout(100);
    tofReady = tof.init() && tof.setMeasurementTimingBudget(33000);
    if (tofReady) {
      tof.startContinuous(50);
      tofReady = tof.last_status == 0;
      lastTofSample = millis();
    }
    nextTofRetry = millis() + 1000;
    return;
  }
  static uint32_t lastPoll = 0;
  if (now - lastPoll < 5) return;
  lastPoll = now;
  uint8_t ready = 0;
  if (!readTof(VL53L0X::RESULT_INTERRUPT_STATUS, &ready, 1)) {
    tofReady = distanceValid = false;
    nextTofRetry = now + 1000;
    return;
  }
  if ((ready & 7) == 0) {
    if (now - lastTofSample > kTofStaleMs) {
      tofReady = distanceValid = false;
      nextTofRetry = now + 1000;
    }
    return;
  }
  uint8_t result[12] = {};
  const bool readOk = readTof(VL53L0X::RESULT_RANGE_STATUS, result, sizeof(result));
  tof.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 1);
  // VL53L0X device range status 11 means a valid range measurement.
  distanceValid = readOk && tof.last_status == 0 && ((result[0] >> 3) & 15) == 11;
  distanceMm = (static_cast<uint16_t>(result[10]) << 8) | result[11];
  distanceValid = distanceValid && distanceMm >= 50 && distanceMm <= 300;
  lastTofSample = now;
  if (!readOk || tof.last_status != 0) {
    tofReady = false;
    nextTofRetry = now + 1000;
  }
}

void handleButtons() {
  static uint32_t pressedAt[3] = {};
  static bool wasDown[3] = {}, held[3] = {};
  const bool down[] = {M5.BtnA.isPressed(), M5.BtnB.isPressed(), M5.BtnC.isPressed()};
  for (int i = 0; i < 3; ++i) {
    if (down[i] && !wasDown[i]) { pressedAt[i] = millis(); held[i] = false; }
    if (down[i] && !held[i] && millis() - pressedAt[i] >= 800) {
      held[i] = true;
      if (i == 0) toggleRun();

      if (i == 2) cycleStepWidth();
    }
    // A long press must not also trigger a short click on release.
    if (!down[i] && wasDown[i] && !held[i]) {
      if (i == 1) desiredSteps = -1;
      if (i == 2) randomizeStep();
    }
    wasDown[i] = down[i];
  }
  for (int i = 0; i < 32 && Serial.available(); ++i) {
    const char c = Serial.read();
    if (c == 'b') desiredSteps = -1;

    else if (c == 'w') cycleStepWidth();
    else if (c == 'r') randomizeStep();
    else if (c == '[') stepStartMm = max(50, stepStartMm - 10);
    else if (c == ']') stepStartMm = min(200, stepStartMm + 10);
    else if (c == '?') help();
  }
}

void updateMotion() {
  targetSteps();
  if (!servoReady && static_cast<int32_t>(millis() - nextServoRetry) >= 0) {
    servoReady = connectServo();
    nextServoRetry = millis() + 1000;
  }
}

void updateServoFeedback() {
  static uint32_t lastFeedback = 0;
  if (!servoReady) { actualStepsValid = false; return; }
  if (millis() - lastFeedback < kServoFeedbackUpdateMs) return;
  lastFeedback = millis();
  const int position = servo.ReadPos(kServoId);
  if (!servo.echoOk || servo.Error != 0 || position < 0 || position > kServoPositionMax) {
    failServo("Position READ failed");
    servoReady = false;
    nextServoRetry = millis() + 1000;
    return;
  }
  actualSteps = (position - kServoZeroPosition) * kServoDirection;
  actualStepsValid = true;
  const int target = targetSteps();
  // Apply once per target transition (or B retry), using this fresh reading.
  // Never accumulate the offset or reverse it as the servo crosses the target.
  if (target != desiredSteps) {
    int command = commandedSteps;
    if (target > actualSteps) command = target + kPositiveOffsetCounts;
    else if (target < actualSteps) command = target - kNegativeOffsetCounts;
    // If already at the target, preserve the previous command.
    if (target != actualSteps && !setServoSteps(command)) {
      servoReady = false;
      nextServoRetry = millis() + 1000;
      return;
    }
    desiredSteps = target;
    Serial.printf("STEP: actual=%d target=%d sent=%d position=%d\n",
                  actualSteps, desiredSteps, commandedSteps,
                  kServoZeroPosition + kServoDirection * commandedSteps);
  }
  servoStatus = actualSteps == desiredSteps ? "Actual matches target" : "Offset applied / holding";
}
void showStatus() {
  static uint32_t lastDisplay = 0;
  if (millis() - lastDisplay < kDisplayUpdateMs) return;
  lastDisplay = millis();
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.setCursor(0, 36);
  const char* mode = !motionEnabled ? "PAUSE / ZERO" : "RUN";
  M5.Lcd.printf("Mode: %-19s\n", mode);
  M5.Lcd.printf("ToF:%4u mm %-12s\n", distanceMm, distanceValid ? "OK" : "INVALID");
  M5.Lcd.printf("Step:%3d-%3d mm On:%d  \n", stepStartMm, stepStartMm + stepWidthMm, onStep);
  M5.Lcd.printf("Target:%+3d steps         \n", onStep ? kStepHeightCounts : 0);
  M5.Lcd.printf("Width:%d cm Noise:OFF     \n", stepWidthMm / 10);
  M5.Lcd.printf("Offset: +%d / -%d           \n", kPositiveOffsetCounts, kNegativeOffsetCounts);
  if (actualStepsValid) M5.Lcd.printf("Actual:%+5d steps       \n", actualSteps);
  else M5.Lcd.print("Actual:  --- steps       \n");
  M5.Lcd.setTextSize(1);
  M5.Lcd.printf("%-40s\n", servoStatus);
  M5.Lcd.printf("Vib:%s Touch:%s                       \n", motorOffVerified ? "OFF" : "ERR", touchReady ? "OK" : "ERR");
  M5.Lcd.setCursor(0, 202);
  M5.Lcd.println("Click B:Retry C:Random position");
  M5.Lcd.println("Hold  A:Run/Pause C:Width");
  M5.Lcd.printf("Actual target: +%d / 0 steps\n", kStepHeightCounts);
}

}  // namespace

void setup() {
  M5.begin(false, false, true, false);
  stopMotorChecked();
  Serial.begin(115200);
  initializeDisplay();
  stopMotorChecked();
  M5.Touch.begin();
  Wire1.setClock(100000);
  Wire1.setTimeOut(20);
  checkInternalDevices();
  Wire.begin(kTofSdaPin, kTofSclPin, 400000);
  Wire.setTimeOut(20);
  tof.setBus(&Wire);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(8, 12);
  M5.Lcd.print("STS3045M ToF:A Servo:C");
  servo.pSerial = &servoSerial;
  servo.IOTimeOut = 20;
  nextServoRetry = millis();

  help();
  Serial.printf("Start: ID=%u PORT C TX=%d RX=%d; zero=%d; step=%.6f deg\n",
                kServoId, kServoTxPin, kServoRxPin, kServoZeroPosition, kDegreesPerStep);
}

void loop() {
  M5.update();
  handleButtons();
  static uint32_t lastCheck = 0;
  if (millis() - lastCheck >= 1000) {
    lastCheck = millis();
    checkInternalDevices();
  }
  updateTof();
  updateMotion();
  updateServoFeedback();
  showStatus();
  delay(1);
}
