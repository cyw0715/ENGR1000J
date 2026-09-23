#include <Wire.h>
#include <VL53L1X.h>

/*
 * Stage A — read-only Mega/TCA/VL53 diagnostic for the four-actuator lander.
 *
 * Canonical wiring preserved from user-supplied 111(1).ino:
 *   Sensors A1..A4 -> TCA9548A channels 0..3
 *   Motor IN1      -> Mega 22,24,26,28
 *   Motor IN2      -> Mega 23,25,27,29
 *   Motor EN       -> Mega 30,31,32,33
 * Verified physical mapping (2026-08-04):
 *   A1=RL, A2=RR, A3=FR, A4=FL
 *   Solver order FL,FR,RL,RR must be remapped to Mega A4,A3,A1,A2.
 *
 * SAFETY: This sketch contains no motor-start function. All EN/IN pins are
 * forced LOW at boot and on every loop iteration. It rejects SET_TARGET.
 */

const uint8_t TCA_ADDRESS = 0x70;
const uint8_t ACTUATOR_COUNT = 4;
const uint8_t SENSOR_CHANNELS[ACTUATOR_COUNT] = {0, 1, 2, 3};
const uint8_t MOTOR_IN1[ACTUATOR_COUNT] = {22, 24, 26, 28};
const uint8_t MOTOR_IN2[ACTUATOR_COUNT] = {23, 25, 27, 29};
const uint8_t MOTOR_EN[ACTUATOR_COUNT] = {30, 31, 32, 33};

const unsigned long SENSOR_STALE_TIMEOUT_MS = 1200;
const unsigned long SENSOR_READ_GAP_MS = 10;
const unsigned long PERIODIC_STATUS_MS = 500;

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

void printStatus()
{
  Serial.print("LANDER_STATUS mode=DIAGNOSTIC_MOTION_LOCKED");
  for (uint8_t i = 0; i < ACTUATOR_COUNT; ++i) {
    Serial.print(" A");
    Serial.print(i + 1);
    Serial.print("_mm=");
    if (sensorValid[i]) Serial.print(currentMM[i]);
    else Serial.print(-1);
    Serial.print(" A");
    Serial.print(i + 1);
    Serial.print("_valid=");
    Serial.print(sensorValid[i] ? 1 : 0);
    Serial.print(" A");
    Serial.print(i + 1);
    Serial.print("_ver=");
    Serial.print(measurementVersion[i]);
  }
  Serial.println();
}

void processCommand(char *command)
{
  if (strcmp(command, "PING") == 0) {
    Serial.println("PONG MEGA_LANDER_DIAGNOSTIC_V1");
    return;
  }
  if (strcmp(command, "STATUS") == 0) {
    printStatus();
    return;
  }
  if (strcmp(command, "STOP") == 0 || strcmp(command, "ESTOP") == 0) {
    hardStopAll();
    Serial.println("OK MOTION_LOCKED ALL_OUTPUTS_LOW");
    return;
  }
  if (strncmp(command, "SET_TARGET", 10) == 0 ||
      strncmp(command, "MOVE", 4) == 0 ||
      strncmp(command, "ARM", 3) == 0) {
    hardStopAll();
    Serial.println("ERR MOTION_DISABLED_STAGE_A");
    return;
  }
  if (strcmp(command, "GET_DISTANCE") == 0) {
    // Backward-compatible harmless response for older P4 pollers.
    printStatus();
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
      commandLength = 0;
      Serial.println("ERR COMMAND_TOO_LONG");
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
  Serial.println("MEGA_LANDER_DIAGNOSTIC_V1 BOOT MOTION_LOCKED ALL_OUTPUTS_LOW");

  Wire.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(25000, true);
  if (!disableAllTCAChannels()) {
    Serial.println("FAULT TCA9548A_NOT_DETECTED MOTION_LOCKED");
    return;
  }

  for (uint8_t i = 0; i < ACTUATOR_COUNT; ++i) {
    if (initializeSensor(i)) {
      Serial.print("SENSOR_READY A");
      Serial.print(i + 1);
      Serial.print(" TCA_CH=");
      Serial.println(SENSOR_CHANNELS[i]);
    } else {
      Serial.print("SENSOR_FAULT A");
      Serial.print(i + 1);
      Serial.print(" TCA_CH=");
      Serial.println(SENSOR_CHANNELS[i]);
    }
  }
  Serial.println("READY COMMANDS=PING,STATUS,STOP,ESTOP MOTION_COMMANDS=REJECTED");
}

void loop()
{
  // Enforce the no-motion invariant continuously, not only at boot.
  hardStopAll();
  updateOneSensor();
  pollCommands();
  if (millis() - lastStatusMS >= PERIODIC_STATUS_MS) {
    lastStatusMS = millis();
    printStatus();
  }
}
