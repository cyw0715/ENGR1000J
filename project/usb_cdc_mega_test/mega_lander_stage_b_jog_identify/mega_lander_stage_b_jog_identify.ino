#include <Wire.h>
#include <VL53L1X.h>

/*
 * Stage B — one-shot, one-leg identification jog for the four-actuator lander.
 * Canonical wiring and motor polarity are preserved from user-supplied 111(1).ino.
 *
 * Safety contract:
 * - Boot: every EN/IN output LOW.
 * - No autonomous movement and no PS2 control.
 * - Every jog requires a fresh exact command: ARM_JOG IDENTIFY.
 * - Arm expires after 10 s and is consumed by one jog attempt.
 * - Exactly one actuator may run, for a fixed 120 ms, then all EN/IN return LOW.
 * - All four VL53L1X sensors must be valid and within 60..250 mm before motion.
 * - ESTOP latches until physical reset.
 * - SET_TARGET/MOVE/ARM are rejected.
 */

const uint8_t TCA_ADDRESS = 0x70;
const uint8_t ACTUATOR_COUNT = 4;
const uint8_t SENSOR_CHANNELS[ACTUATOR_COUNT] = {0, 1, 2, 3};
const uint8_t MOTOR_IN1[ACTUATOR_COUNT] = {22, 24, 26, 28};
const uint8_t MOTOR_IN2[ACTUATOR_COUNT] = {23, 25, 27, 29};
const uint8_t MOTOR_EN[ACTUATOR_COUNT] = {30, 31, 32, 33};

const uint16_t SENSOR_RETRACT_FLOOR_MM = 65;
const uint16_t SENSOR_EXTEND_CEILING_MM = 400;
const unsigned long SENSOR_STALE_TIMEOUT_MS = 1200;
const unsigned long SENSOR_READ_GAP_MS = 10;
const unsigned long PERIODIC_STATUS_MS = 500;
const unsigned long JOG_ARM_WINDOW_MS = 10000;
const unsigned long JOG_DURATION_MS = 120;
const unsigned long POST_JOG_SETTLE_MS = 180;

VL53L1X sensors[ACTUATOR_COUNT];
uint16_t currentMM[ACTUATOR_COUNT] = {0, 0, 0, 0};
bool sensorValid[ACTUATOR_COUNT] = {false, false, false, false};
unsigned long lastValidMS[ACTUATOR_COUNT] = {0, 0, 0, 0};
uint32_t measurementVersion[ACTUATOR_COUNT] = {0, 0, 0, 0};
uint8_t nextSensor = 0;
unsigned long lastSensorReadMS = 0;
unsigned long lastStatusMS = 0;
char commandLine[96];
size_t commandLength = 0;
bool jogArmed = false;
unsigned long jogArmTimeMS = 0;
bool estopLatched = false;

void hardStopAll()
{
  for (uint8_t i = 0; i < ACTUATOR_COUNT; ++i) {
    digitalWrite(MOTOR_IN1[i], LOW);
    digitalWrite(MOTOR_IN2[i], LOW);
    digitalWrite(MOTOR_EN[i], LOW);
  }
}

bool selectTCAChannel(uint8_t channel)
{
  if (channel > 7) return false;
  Wire.beginTransmission(TCA_ADDRESS);
  Wire.write((uint8_t)(1U << channel));
  if (Wire.endTransmission() != 0) return false;
  delayMicroseconds(300);
  return true;
}

bool disableAllTCAChannels()
{
  Wire.beginTransmission(TCA_ADDRESS);
  Wire.write((uint8_t)0x00);
  return Wire.endTransmission() == 0;
}

bool initializeSensor(uint8_t index)
{
  if (!selectTCAChannel(SENSOR_CHANNELS[index])) return false;
  delay(50);
  sensors[index].setTimeout(500);
  if (!sensors[index].init()) return false;
  sensors[index].setDistanceMode(VL53L1X::Short);
  if (!sensors[index].setMeasurementTimingBudget(50000)) return false;
  sensors[index].startContinuous(60);
  return true;
}

bool readSensor(uint8_t index)
{
  bool valid = false;
  uint16_t distance = 0;
  if (selectTCAChannel(SENSOR_CHANNELS[index])) {
    distance = sensors[index].read();
    if (!sensors[index].timeoutOccurred() &&
        sensors[index].ranging_data.range_status == VL53L1X::RangeValid &&
        distance > 0 && distance <= 4000) {
      valid = true;
    }
  }
  if (valid) {
    currentMM[index] = distance;
    sensorValid[index] = true;
    lastValidMS[index] = millis();
    measurementVersion[index]++;
    return true;
  }
  if (lastValidMS[index] == 0 ||
      millis() - lastValidMS[index] > SENSOR_STALE_TIMEOUT_MS) {
    sensorValid[index] = false;
  }
  return false;
}

void updateOneSensor()
{
  if (millis() - lastSensorReadMS < SENSOR_READ_GAP_MS) return;
  lastSensorReadMS = millis();
  readSensor(nextSensor);
  nextSensor = (uint8_t)((nextSensor + 1U) % ACTUATOR_COUNT);
}

bool allSensorsValid()
{
  for (uint8_t i = 0; i < ACTUATOR_COUNT; ++i) {
    if (!sensorValid[i]) return false;
  }
  return true;
}

void printStatus()
{
  Serial.print("LANDER_STATUS mode=JOG_IDENTIFY_LOCKED jog_armed=");
  Serial.print(jogArmed ? 1 : 0);
  Serial.print(" estop=");
  Serial.print(estopLatched ? 1 : 0);
  for (uint8_t i = 0; i < ACTUATOR_COUNT; ++i) {
    Serial.print(" A"); Serial.print(i + 1); Serial.print("_mm=");
    if (sensorValid[i]) Serial.print(currentMM[i]); else Serial.print(-1);
    Serial.print(" A"); Serial.print(i + 1); Serial.print("_valid=");
    Serial.print(sensorValid[i] ? 1 : 0);
    Serial.print(" A"); Serial.print(i + 1); Serial.print("_ver=");
    Serial.print(measurementVersion[i]);
  }
  Serial.println();
}

bool parseActuator(const char *token, uint8_t &index)
{
  if (token == NULL || token[0] != 'A' || token[2] != '\0' ||
      token[1] < '1' || token[1] > '4') return false;
  index = (uint8_t)(token[1] - '1');
  return true;
}

void runOneShotJog(uint8_t index, bool retract)
{
  // Consume authorization before any further check so retries require re-arming.
  jogArmed = false;
  hardStopAll();
  if (estopLatched) {
    Serial.println("ERR ESTOP_LATCHED_RESET_REQUIRED");
    return;
  }
  if (!allSensorsValid()) {
    Serial.println("ERR SENSOR_GATE_REQUIRES_ALL_VALID");
    return;
  }
  if ((retract && currentMM[index] <= SENSOR_RETRACT_FLOOR_MM) ||
      (!retract && currentMM[index] >= SENSOR_EXTEND_CEILING_MM)) {
    Serial.println("ERR DIRECTIONAL_SENSOR_LIMIT");
    return;
  }

  const uint16_t before = currentMM[index];
  Serial.print("JOG_START actuator=A"); Serial.print(index + 1);
  Serial.print(" polarity="); Serial.print(retract ? "RETRACT" : "EXTEND");
  Serial.print(" before_mm="); Serial.print(before);
  Serial.print(" duration_ms="); Serial.println(JOG_DURATION_MS);

  // Reference polarity from 111(1).ino. Break-before-make is guaranteed by hardStopAll().
  digitalWrite(MOTOR_IN1[index], retract ? LOW : HIGH);
  digitalWrite(MOTOR_IN2[index], retract ? HIGH : LOW);
  digitalWrite(MOTOR_EN[index], HIGH);

  const unsigned long started = millis();
  while (millis() - started < JOG_DURATION_MS) {
    // Local elapsed-time bound is the authority; no host heartbeat is needed to stop.
    delay(1);
  }
  hardStopAll();

  delay(POST_JOG_SETTLE_MS);
  bool afterValid = readSensor(index);
  int after = afterValid ? (int)currentMM[index] : -1;
  int delta = afterValid ? after - (int)before : 0;
  Serial.print("JOG_RESULT actuator=A"); Serial.print(index + 1);
  Serial.print(" polarity="); Serial.print(retract ? "RETRACT" : "EXTEND");
  Serial.print(" before_mm="); Serial.print(before);
  Serial.print(" after_mm="); Serial.print(after);
  Serial.print(" delta_mm="); Serial.print(delta);
  Serial.println(" outputs=ALL_LOW arm_consumed=1");
}

void processCommand(char *command)
{
  if (strcmp(command, "PING") == 0) {
    Serial.println("PONG MEGA_LANDER_JOG_IDENTIFY_V1");
    return;
  }
  if (strcmp(command, "STATUS") == 0) {
    printStatus();
    return;
  }
  if (strcmp(command, "STOP") == 0) {
    hardStopAll(); jogArmed = false;
    Serial.println("OK STOPPED ALL_OUTPUTS_LOW JOG_DISARMED");
    return;
  }
  if (strcmp(command, "ESTOP") == 0) {
    hardStopAll(); jogArmed = false; estopLatched = true;
    Serial.println("OK ESTOP_LATCHED ALL_OUTPUTS_LOW RESET_REQUIRED");
    return;
  }
  if (strcmp(command, "ARM_JOG IDENTIFY") == 0) {
    hardStopAll();
    if (estopLatched) {
      Serial.println("ERR ESTOP_LATCHED_RESET_REQUIRED");
    } else if (!allSensorsValid()) {
      Serial.println("ERR SENSOR_GATE_REQUIRES_ALL_VALID");
    } else {
      jogArmed = true; jogArmTimeMS = millis();
      Serial.println("OK JOG_ARMED_ONE_SHOT_EXPIRES_MS=10000");
    }
    return;
  }
  if (strncmp(command, "JOG ", 4) == 0) {
    char *save = NULL;
    char *actuator = strtok_r(command + 4, " ", &save);
    char *polarity = strtok_r(NULL, " ", &save);
    char *extra = strtok_r(NULL, " ", &save);
    uint8_t index = 0;
    if (!jogArmed || millis() - jogArmTimeMS > JOG_ARM_WINDOW_MS) {
      jogArmed = false;
      Serial.println("ERR JOG_NOT_ARMED");
    } else if (!parseActuator(actuator, index) || polarity == NULL || extra != NULL ||
               (strcmp(polarity, "RETRACT") != 0 && strcmp(polarity, "EXTEND") != 0)) {
      jogArmed = false;
      Serial.println("ERR FORMAT_JOG_A1_TO_A4_RETRACT_OR_EXTEND ARM_CONSUMED");
    } else {
      runOneShotJog(index, strcmp(polarity, "RETRACT") == 0);
    }
    return;
  }
  if (strncmp(command, "SET_TARGET", 10) == 0 ||
      strncmp(command, "MOVE", 4) == 0 || strcmp(command, "ARM") == 0) {
    hardStopAll(); jogArmed = false;
    Serial.println("ERR CONTINUOUS_MOTION_DISABLED_IDENTIFICATION_ONLY");
    return;
  }
  Serial.println("ERR UNKNOWN_COMMAND");
}

void pollCommands()
{
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      commandLine[commandLength] = '\0';
      if (commandLength > 0) processCommand(commandLine);
      commandLength = 0;
    } else if (commandLength < sizeof(commandLine) - 1) {
      commandLine[commandLength++] = ch;
    } else {
      commandLength = 0; hardStopAll(); jogArmed = false;
      Serial.println("ERR COMMAND_TOO_LONG JOG_DISARMED");
    }
  }
}

void setup()
{
  Serial.begin(115200);
  for (uint8_t i = 0; i < ACTUATOR_COUNT; ++i) {
    pinMode(MOTOR_IN1[i], OUTPUT);
    pinMode(MOTOR_IN2[i], OUTPUT);
    pinMode(MOTOR_EN[i], OUTPUT);
  }
  hardStopAll();
  delay(1500);
  Serial.println("MEGA_LANDER_JOG_IDENTIFY_V1 BOOT LOCKED ALL_OUTPUTS_LOW");

  Wire.begin(); Wire.setClock(100000); Wire.setWireTimeout(25000, true);
  if (!disableAllTCAChannels()) {
    Serial.println("FAULT TCA9548A_NOT_DETECTED MOTION_LOCKED");
    return;
  }
  for (uint8_t i = 0; i < ACTUATOR_COUNT; ++i) {
    Serial.print(initializeSensor(i) ? "SENSOR_READY A" : "SENSOR_FAULT A");
    Serial.print(i + 1); Serial.print(" TCA_CH="); Serial.println(SENSOR_CHANNELS[i]);
  }
  delay(500);
  for (uint8_t i = 0; i < ACTUATOR_COUNT; ++i) readSensor(i);
  Serial.println("READY ONE_SHOT_JOG_REQUIRES_ARM_JOG_IDENTIFY ALL_OTHER_MOTION_REJECTED");
}

void loop()
{
  hardStopAll();
  if (jogArmed && millis() - jogArmTimeMS > JOG_ARM_WINDOW_MS) {
    jogArmed = false;
    Serial.println("JOG_ARM_EXPIRED");
  }
  updateOneSensor();
  pollCommands();
  if (millis() - lastStatusMS >= PERIODIC_STATUS_MS) {
    lastStatusMS = millis(); printStatus();
  }
}
