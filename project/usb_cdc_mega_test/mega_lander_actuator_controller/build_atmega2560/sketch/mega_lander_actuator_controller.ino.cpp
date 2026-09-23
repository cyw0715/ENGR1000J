#include <Arduino.h>
#line 1 "E:\\Chenyuewei\\University\\Freshman\\summer\\ENGR1000\\p2\\project\\usb_cdc_mega_test\\mega_lander_actuator_controller\\mega_lander_actuator_controller.ino"
/*
 * Mars Lander Mega 2560 -- four corner linear-actuator controller.
 *
 * Derived from the supplied sketch's verified building blocks:
 * - four H-bridge channels: IN1/IN2/EN on D22..D33;
 * - four VL53L1X feedback sensors through TCA9548A channels 0..3;
 * - range validation, repeated confirmations, per-leg motion timeout, and
 *   repeated-read failure shutdown.
 *
 * This is a NEW, separate application. It is intentionally not flashed here.
 * It is safe-by-default: boot DISARMED, all bridge enables LOW, and no SET_TARGET
 * command can move hardware until ARM + a valid heartbeat have been received.
 *
 * Hardware mapping (must be checked against the actual driver board before flash):
 *   FL: IN1 D22, IN2 D23, EN D30, VL53L1X TCA channel 0
 *   FR: IN1 D24, IN2 D25, EN D31, VL53L1X TCA channel 1
 *   RL: IN1 D26, IN2 D27, EN D32, VL53L1X TCA channel 2
 *   RR: IN1 D28, IN2 D29, EN D33, VL53L1X TCA channel 3
 *
 * USB serial protocol, LF-terminated ASCII, 115200 8-N-1:
 *   PING
 *   ARM                 arm only after all four feedback sensors are valid
 *   HEARTBEAT           required while moving; controller stops after 500 ms
 *   SET_TARGET FL FR RL RR
 *                       four actuator-position targets in calibrated mm, each 250..450
 *   STATUS
 *   STOP                stop now; remain armed
 *   ESTOP               latch fault; only a hardware reset clears it
 *
 * IMPORTANT:
 * - VL53L1X distance is NOT automatically equal to actuator length. Set
 *   POSITION_OFFSET_MM[] and POSITION_INCREASES_WHEN_EXTENDING[] from physical
 *   calibration before setting POSITION_CALIBRATION_VALID true.
 * - The PC/P4 must not send commands directly to a motor driver. Commands must
 *   pass through the tested P4<->Mega bridge and the Mega must retain its own
 *   watchdog, limits, and ESTOP response.
 */

#include <Wire.h>
#include <VL53L1X.h>
#include <stdlib.h>
#include <string.h>

constexpr uint8_t ACTUATOR_COUNT = 4;
constexpr uint8_t TCA_ADDRESS = 0x70;
constexpr uint8_t SENSOR_CHANNELS[ACTUATOR_COUNT] = {0, 1, 2, 3};
constexpr uint8_t MOTOR_IN1[ACTUATOR_COUNT] = {22, 24, 26, 28};
constexpr uint8_t MOTOR_IN2[ACTUATOR_COUNT] = {23, 25, 27, 29};
constexpr uint8_t MOTOR_EN[ACTUATOR_COUNT] = {30, 31, 32, 33};
constexpr const char *LEG_NAME[ACTUATOR_COUNT] = {"FL", "FR", "RL", "RR"};

// These values implement the lander solver's axial actuator travel contract.
constexpr int16_t LEG_MIN_MM = 250;
constexpr int16_t LEG_MAX_MM = 450;
constexpr int16_t TARGET_TOLERANCE_MM = 3;
constexpr uint8_t TARGET_CONFIRMATIONS = 3;
constexpr uint8_t MAX_CONSECUTIVE_SENSOR_FAILURES = 5;
constexpr uint32_t SENSOR_INTERVAL_MS = 60;
constexpr uint32_t LEG_MOVE_TIMEOUT_MS = 30000UL;
constexpr uint32_t WATCHDOG_TIMEOUT_MS = 500UL;
constexpr uint16_t SENSOR_MIN_MM = 20;
constexpr uint16_t SENSOR_MAX_MM = 4000;
constexpr bool POSITION_CALIBRATION_VALID = false;  // Set true only after each leg is measured.

// Calibrated actuator_length_mm = distance_mm + POSITION_OFFSET_MM[i].
// Do not guess these values. The default makes ARM fail safely.
const int16_t POSITION_OFFSET_MM[ACTUATOR_COUNT] = {0, 0, 0, 0};
const bool POSITION_INCREASES_WHEN_EXTENDING[ACTUATOR_COUNT] = {true, true, true, true};

VL53L1X sensors[ACTUATOR_COUNT];

enum ControllerState : uint8_t {
  DISARMED,
  ARMED_IDLE,
  MOVING,
  FAULT_LATCHED,
};

ControllerState controllerState = DISARMED;
int16_t targetLengthMM[ACTUATOR_COUNT] = {LEG_MIN_MM, LEG_MIN_MM, LEG_MIN_MM, LEG_MIN_MM};
int16_t currentLengthMM[ACTUATOR_COUNT] = {0, 0, 0, 0};
bool sensorValid[ACTUATOR_COUNT] = {false, false, false, false};
uint8_t confirmationCount[ACTUATOR_COUNT] = {0, 0, 0, 0};
uint8_t sensorFailureCount[ACTUATOR_COUNT] = {0, 0, 0, 0};
uint32_t moveStartMS = 0;
uint32_t lastSensorPollMS = 0;
uint32_t lastHeartbeatMS = 0;
char commandLine[96];
size_t commandLength = 0;

#line 91 "E:\\Chenyuewei\\University\\Freshman\\summer\\ENGR1000\\p2\\project\\usb_cdc_mega_test\\mega_lander_actuator_controller\\mega_lander_actuator_controller.ino"
const char * stateName(ControllerState state);
#line 101 "E:\\Chenyuewei\\University\\Freshman\\summer\\ENGR1000\\p2\\project\\usb_cdc_mega_test\\mega_lander_actuator_controller\\mega_lander_actuator_controller.ino"
bool selectTCAChannel(uint8_t channel);
#line 110 "E:\\Chenyuewei\\University\\Freshman\\summer\\ENGR1000\\p2\\project\\usb_cdc_mega_test\\mega_lander_actuator_controller\\mega_lander_actuator_controller.ino"
void stopActuator(uint8_t index);
#line 115 "E:\\Chenyuewei\\University\\Freshman\\summer\\ENGR1000\\p2\\project\\usb_cdc_mega_test\\mega_lander_actuator_controller\\mega_lander_actuator_controller.ino"
void stopAll();
#line 122 "E:\\Chenyuewei\\University\\Freshman\\summer\\ENGR1000\\p2\\project\\usb_cdc_mega_test\\mega_lander_actuator_controller\\mega_lander_actuator_controller.ino"
void driveActuator(uint8_t index, bool extend);
#line 131 "E:\\Chenyuewei\\University\\Freshman\\summer\\ENGR1000\\p2\\project\\usb_cdc_mega_test\\mega_lander_actuator_controller\\mega_lander_actuator_controller.ino"
void enterFault(const char *reason);
#line 138 "E:\\Chenyuewei\\University\\Freshman\\summer\\ENGR1000\\p2\\project\\usb_cdc_mega_test\\mega_lander_actuator_controller\\mega_lander_actuator_controller.ino"
bool initializeSensor(uint8_t index);
#line 148 "E:\\Chenyuewei\\University\\Freshman\\summer\\ENGR1000\\p2\\project\\usb_cdc_mega_test\\mega_lander_actuator_controller\\mega_lander_actuator_controller.ino"
bool readActuatorPosition(uint8_t index, int16_t &positionMM);
#line 161 "E:\\Chenyuewei\\University\\Freshman\\summer\\ENGR1000\\p2\\project\\usb_cdc_mega_test\\mega_lander_actuator_controller\\mega_lander_actuator_controller.ino"
bool updateAllPositions();
#line 178 "E:\\Chenyuewei\\University\\Freshman\\summer\\ENGR1000\\p2\\project\\usb_cdc_mega_test\\mega_lander_actuator_controller\\mega_lander_actuator_controller.ino"
bool allPositionsSafeToArm();
#line 199 "E:\\Chenyuewei\\University\\Freshman\\summer\\ENGR1000\\p2\\project\\usb_cdc_mega_test\\mega_lander_actuator_controller\\mega_lander_actuator_controller.ino"
bool targetReached(uint8_t index);
#line 203 "E:\\Chenyuewei\\University\\Freshman\\summer\\ENGR1000\\p2\\project\\usb_cdc_mega_test\\mega_lander_actuator_controller\\mega_lander_actuator_controller.ino"
void updateMotion();
#line 254 "E:\\Chenyuewei\\University\\Freshman\\summer\\ENGR1000\\p2\\project\\usb_cdc_mega_test\\mega_lander_actuator_controller\\mega_lander_actuator_controller.ino"
void reportStatus();
#line 278 "E:\\Chenyuewei\\University\\Freshman\\summer\\ENGR1000\\p2\\project\\usb_cdc_mega_test\\mega_lander_actuator_controller\\mega_lander_actuator_controller.ino"
bool parseTargets(char *arguments);
#line 301 "E:\\Chenyuewei\\University\\Freshman\\summer\\ENGR1000\\p2\\project\\usb_cdc_mega_test\\mega_lander_actuator_controller\\mega_lander_actuator_controller.ino"
void processCommand(char *command);
#line 364 "E:\\Chenyuewei\\University\\Freshman\\summer\\ENGR1000\\p2\\project\\usb_cdc_mega_test\\mega_lander_actuator_controller\\mega_lander_actuator_controller.ino"
void pollCommands();
#line 381 "E:\\Chenyuewei\\University\\Freshman\\summer\\ENGR1000\\p2\\project\\usb_cdc_mega_test\\mega_lander_actuator_controller\\mega_lander_actuator_controller.ino"
void setup();
#line 409 "E:\\Chenyuewei\\University\\Freshman\\summer\\ENGR1000\\p2\\project\\usb_cdc_mega_test\\mega_lander_actuator_controller\\mega_lander_actuator_controller.ino"
void loop();
#line 91 "E:\\Chenyuewei\\University\\Freshman\\summer\\ENGR1000\\p2\\project\\usb_cdc_mega_test\\mega_lander_actuator_controller\\mega_lander_actuator_controller.ino"
const char *stateName(ControllerState state) {
  switch (state) {
    case DISARMED: return "DISARMED";
    case ARMED_IDLE: return "ARMED_IDLE";
    case MOVING: return "MOVING";
    case FAULT_LATCHED: return "FAULT_LATCHED";
  }
  return "UNKNOWN";
}

bool selectTCAChannel(uint8_t channel) {
  if (channel > 7) return false;
  Wire.beginTransmission(TCA_ADDRESS);
  Wire.write((uint8_t)(1U << channel));
  if (Wire.endTransmission() != 0) return false;
  delayMicroseconds(300);
  return true;
}

void stopActuator(uint8_t index) {
  digitalWrite(MOTOR_IN1[index], LOW);
  digitalWrite(MOTOR_IN2[index], LOW);
}

void stopAll() {
  for (uint8_t i = 0; i < ACTUATOR_COUNT; ++i) {
    stopActuator(i);
    digitalWrite(MOTOR_EN[i], LOW);
  }
}

void driveActuator(uint8_t index, bool extend) {
  // Break-before-make: no H-bridge direction reversal while an output is active.
  stopActuator(index);
  delay(5);
  digitalWrite(MOTOR_EN[index], HIGH);
  digitalWrite(MOTOR_IN1[index], extend ? HIGH : LOW);
  digitalWrite(MOTOR_IN2[index], extend ? LOW : HIGH);
}

void enterFault(const char *reason) {
  stopAll();
  controllerState = FAULT_LATCHED;
  Serial.print("FAULT reason=");
  Serial.println(reason);
}

bool initializeSensor(uint8_t index) {
  if (!selectTCAChannel(SENSOR_CHANNELS[index])) return false;
  sensors[index].setTimeout(500);
  if (!sensors[index].init()) return false;
  sensors[index].setDistanceMode(VL53L1X::Short);
  if (!sensors[index].setMeasurementTimingBudget(50000)) return false;
  sensors[index].startContinuous(SENSOR_INTERVAL_MS);
  return true;
}

bool readActuatorPosition(uint8_t index, int16_t &positionMM) {
  if (!selectTCAChannel(SENSOR_CHANNELS[index])) return false;
  const uint16_t rawDistance = sensors[index].read();
  if (sensors[index].timeoutOccurred()) return false;
  if (sensors[index].ranging_data.range_status != VL53L1X::RangeValid) return false;
  if (rawDistance < SENSOR_MIN_MM || rawDistance > SENSOR_MAX_MM) return false;

  const int32_t calibrated = (int32_t)rawDistance + POSITION_OFFSET_MM[index];
  if (calibrated < -32768 || calibrated > 32767) return false;
  positionMM = (int16_t)calibrated;
  return true;
}

bool updateAllPositions() {
  bool allValid = true;
  for (uint8_t i = 0; i < ACTUATOR_COUNT; ++i) {
    int16_t position = 0;
    if (readActuatorPosition(i, position)) {
      currentLengthMM[i] = position;
      sensorValid[i] = true;
      sensorFailureCount[i] = 0;
    } else {
      sensorValid[i] = false;
      ++sensorFailureCount[i];
      allValid = false;
    }
  }
  return allValid;
}

bool allPositionsSafeToArm() {
  if (!POSITION_CALIBRATION_VALID) {
    Serial.println("ERR position_calibration_invalid");
    return false;
  }
  if (!updateAllPositions()) {
    Serial.println("ERR feedback_not_valid");
    return false;
  }
  for (uint8_t i = 0; i < ACTUATOR_COUNT; ++i) {
    if (currentLengthMM[i] < LEG_MIN_MM || currentLengthMM[i] > LEG_MAX_MM) {
      Serial.print("ERR feedback_out_of_travel leg=");
      Serial.print(LEG_NAME[i]);
      Serial.print(" length_mm=");
      Serial.println(currentLengthMM[i]);
      return false;
    }
  }
  return true;
}

bool targetReached(uint8_t index) {
  return abs(currentLengthMM[index] - targetLengthMM[index]) <= TARGET_TOLERANCE_MM;
}

void updateMotion() {
  if (controllerState != MOVING) return;

  const uint32_t now = millis();
  if ((uint32_t)(now - lastHeartbeatMS) > WATCHDOG_TIMEOUT_MS) {
    enterFault("heartbeat_timeout");
    return;
  }
  if ((uint32_t)(now - moveStartMS) > LEG_MOVE_TIMEOUT_MS) {
    enterFault("move_timeout");
    return;
  }
  if ((uint32_t)(now - lastSensorPollMS) < SENSOR_INTERVAL_MS) return;
  lastSensorPollMS = now;

  updateAllPositions();
  bool allReached = true;
  for (uint8_t i = 0; i < ACTUATOR_COUNT; ++i) {
    if (sensorFailureCount[i] >= MAX_CONSECUTIVE_SENSOR_FAILURES) {
      enterFault("position_sensor_failure");
      return;
    }
    if (!sensorValid[i]) {
      stopActuator(i);
      allReached = false;
      continue;
    }
    if (currentLengthMM[i] < LEG_MIN_MM || currentLengthMM[i] > LEG_MAX_MM) {
      enterFault("measured_travel_limit");
      return;
    }

    if (targetReached(i)) {
      ++confirmationCount[i];
      stopActuator(i);
    } else {
      confirmationCount[i] = 0;
      const bool lengthMustIncrease = currentLengthMM[i] < targetLengthMM[i];
      const bool extend = POSITION_INCREASES_WHEN_EXTENDING[i] ? lengthMustIncrease : !lengthMustIncrease;
      driveActuator(i, extend);
    }
    if (confirmationCount[i] < TARGET_CONFIRMATIONS) allReached = false;
  }

  if (allReached) {
    stopAll();
    controllerState = ARMED_IDLE;
    Serial.println("DONE targets_reached=true");
  }
}

void reportStatus() {
  Serial.print("STATUS state=");
  Serial.print(stateName(controllerState));
  Serial.print(" calibration_valid=");
  Serial.print(POSITION_CALIBRATION_VALID ? "true" : "false");
  Serial.print(" heartbeat_age_ms=");
  Serial.print((uint32_t)(millis() - lastHeartbeatMS));
  for (uint8_t i = 0; i < ACTUATOR_COUNT; ++i) {
    Serial.print(" ");
    Serial.print(LEG_NAME[i]);
    Serial.print("_mm=");
    Serial.print(currentLengthMM[i]);
    Serial.print(" target_");
    Serial.print(LEG_NAME[i]);
    Serial.print("_mm=");
    Serial.print(targetLengthMM[i]);
    Serial.print(" valid_");
    Serial.print(LEG_NAME[i]);
    Serial.print("=");
    Serial.print(sensorValid[i] ? "true" : "false");
  }
  Serial.println();
}

bool parseTargets(char *arguments) {
  long values[ACTUATOR_COUNT];
  for (uint8_t i = 0; i < ACTUATOR_COUNT; ++i) {
    char *token = (i == 0) ? strtok(arguments, " ") : strtok(nullptr, " ");
    if (token == nullptr) {
      Serial.println("ERR set_target_requires_FL_FR_RL_RR");
      return false;
    }
    char *end = nullptr;
    values[i] = strtol(token, &end, 10);
    if (*token == '\0' || *end != '\0' || values[i] < LEG_MIN_MM || values[i] > LEG_MAX_MM) {
      Serial.println("ERR targets_out_of_range");
      return false;
    }
  }
  if (strtok(nullptr, " ") != nullptr) {
    Serial.println("ERR set_target_requires_exactly_four_values");
    return false;
  }
  for (uint8_t i = 0; i < ACTUATOR_COUNT; ++i) targetLengthMM[i] = (int16_t)values[i];
  return true;
}

void processCommand(char *command) {
  if (strcmp(command, "PING") == 0) {
    Serial.println("PONG");
    return;
  }
  if (strcmp(command, "STATUS") == 0) {
    reportStatus();
    return;
  }
  if (strcmp(command, "ESTOP") == 0) {
    enterFault("software_estop");
    return;
  }
  if (strcmp(command, "STOP") == 0) {
    stopAll();
    if (controllerState != FAULT_LATCHED) controllerState = ARMED_IDLE;
    Serial.println("STOPPED");
    return;
  }
  if (strcmp(command, "ARM") == 0) {
    if (controllerState == FAULT_LATCHED) {
      Serial.println("ERR reset_required_after_fault");
      return;
    }
    if (!allPositionsSafeToArm()) return;
    controllerState = ARMED_IDLE;
    lastHeartbeatMS = millis();
    Serial.println("ARMED");
    return;
  }
  if (strcmp(command, "HEARTBEAT") == 0) {
    if (controllerState == DISARMED || controllerState == FAULT_LATCHED) {
      Serial.println("ERR heartbeat_ignored_not_armed");
      return;
    }
    lastHeartbeatMS = millis();
    Serial.println("HEARTBEAT_OK");
    return;
  }
  if (strncmp(command, "SET_TARGET ", 11) == 0) {
    if (controllerState != ARMED_IDLE) {
      Serial.println("ERR controller_not_armed_idle");
      return;
    }
    if (!parseTargets(command + 11)) return;
    if (!allPositionsSafeToArm()) return;
    memset(confirmationCount, 0, sizeof(confirmationCount));
    memset(sensorFailureCount, 0, sizeof(sensorFailureCount));
    moveStartMS = millis();
    lastHeartbeatMS = moveStartMS;
    lastSensorPollMS = 0;
    controllerState = MOVING;
    Serial.print("MOVING targets_mm=");
    Serial.print(targetLengthMM[0]); Serial.print(",");
    Serial.print(targetLengthMM[1]); Serial.print(",");
    Serial.print(targetLengthMM[2]); Serial.print(",");
    Serial.println(targetLengthMM[3]);
    return;
  }
  Serial.print("ERR unknown_command=");
  Serial.println(command);
}

void pollCommands() {
  while (Serial.available() > 0) {
    const char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      commandLine[commandLength] = '\0';
      if (commandLength > 0) processCommand(commandLine);
      commandLength = 0;
    } else if (commandLength < sizeof(commandLine) - 1) {
      commandLine[commandLength++] = ch;
    } else {
      commandLength = 0;
      Serial.println("ERR command_too_long");
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(800);

  for (uint8_t i = 0; i < ACTUATOR_COUNT; ++i) {
    pinMode(MOTOR_IN1[i], OUTPUT);
    pinMode(MOTOR_IN2[i], OUTPUT);
    pinMode(MOTOR_EN[i], OUTPUT);
    digitalWrite(MOTOR_EN[i], LOW);
  }
  stopAll();

  Wire.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(25000, true);

  bool sensorsReady = true;
  for (uint8_t i = 0; i < ACTUATOR_COUNT; ++i) {
    if (!initializeSensor(i)) sensorsReady = false;
  }

  Serial.print("MEGA_LANDER_READY state=DISARMED sensors_ready=");
  Serial.print(sensorsReady ? "true" : "false");
  Serial.print(" calibration_valid=");
  Serial.println(POSITION_CALIBRATION_VALID ? "true" : "false");
  if (!sensorsReady) enterFault("sensor_initialization_failed");
}

void loop() {
  pollCommands();
  updateMotion();
}

