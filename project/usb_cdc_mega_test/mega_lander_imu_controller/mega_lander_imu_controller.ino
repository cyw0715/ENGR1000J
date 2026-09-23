#include <Wire.h>
#include <VL53L1X.h>

/*
 * Mega Lander IMU Controller V2 — continuous, feedback-driven, non-blocking.
 * Canonical pins/polarity from user-proven 111(1).ino.
 * Verified physical mapping: A1=RL, A2=RR, A3=FR, A4=FL.
 * Solver order FL,FR,RL,RR -> Mega A4,A3,A1,A2.
 * A1 RETRACT was hardware-verified to decrease its VL53 reading.
 */

const uint8_t TCA_ADDRESS = 0x70;
const uint8_t ACTUATOR_COUNT = 4;
const uint8_t SENSOR_CHANNELS[ACTUATOR_COUNT] = {0, 1, 2, 3};
const uint8_t MOTOR_IN1[ACTUATOR_COUNT] = {22, 24, 26, 28};
const uint8_t MOTOR_IN2[ACTUATOR_COUNT] = {23, 25, 27, 29};
const uint8_t MOTOR_EN[ACTUATOR_COUNT]  = {30, 31, 32, 33};
const uint8_t SOLVER_TO_MEGA[ACTUATOR_COUNT] = {3, 2, 0, 1};

// HC-SR04 is physically mounted beside the downward camera.
// D52 is PB1 / PCINT1 on ATmega2560, so Echo is captured by pin-change ISR
// without blocking the four-leg feedback loop.
const uint8_t SR04_TRIG_PIN = 53;
const uint8_t SR04_ECHO_PIN = 52;
const unsigned long SR04_PERIOD_MS = 100;
const unsigned long SR04_ECHO_TIMEOUT_US = 30000UL;
const int16_t SR04_MIN_MM = 20;
const int16_t SR04_MAX_MM = 4000;

const unsigned long SENSOR_STALE_TIMEOUT_MS = 1200;
const unsigned long SENSOR_READ_GAP_MS = 10;
const unsigned long PERIODIC_STATUS_MS = 200;
const unsigned long HEARTBEAT_TIMEOUT_MS = 2000;
const unsigned long REVERSAL_DEAD_TIME_MS = 120;
const int16_t SENSOR_RANGE_MIN_MM = 50;
const int16_t SENSOR_RANGE_MAX_MM = 400;
const int16_t DELTA_PROTOCOL_MAX_MM = 350;
const int16_t TARGET_TOLERANCE_MM = 1;
const int16_t DIRECTION_PROGRESS_MIN_MM = 1;
const unsigned long DIRECTION_VERIFY_MS = 120;

// All four polarities were hardware-verified with bounded 120 ms RETRACT jogs:
// A1 257→255, A2 99→98, A3 165→163, A4 234→232 mm.
bool directionVerified[ACTUATOR_COUNT] = {true, true, true, true};

enum MegaState : uint8_t {
  STATE_BOOT, STATE_DISARMED, STATE_ARMED, STATE_ACTIVE, STATE_FAULT, STATE_ESTOP
};
enum OutputDirection : int8_t { DIR_IDLE = 0, DIR_RETRACT = -1, DIR_EXTEND = 1 };

VL53L1X sensors[ACTUATOR_COUNT];
uint16_t currentMM[ACTUATOR_COUNT] = {0, 0, 0, 0};
int16_t filteredMM[ACTUATOR_COUNT] = {0, 0, 0, 0};
bool filterInitialized[ACTUATOR_COUNT] = {false, false, false, false};
bool sensorValid[ACTUATOR_COUNT] = {false, false, false, false};
unsigned long lastValidMS[ACTUATOR_COUNT] = {0, 0, 0, 0};
uint32_t measurementVersion[ACTUATOR_COUNT] = {0, 0, 0, 0};
uint8_t nextSensor = 0;
unsigned long lastSensorReadMS = 0;
unsigned long lastStatusMS = 0;

MegaState state = STATE_BOOT;
bool estopLatched = false;
uint32_t lastCommandSeq = 0;
unsigned long lastHeartbeatMS = 0;
uint32_t faultCount = 0;
char faultReason[32] = "none";

// Solver-order targets; baselines are stored in physical Mega order.
int16_t armBaselineA[ACTUATOR_COUNT] = {0, 0, 0, 0};
int16_t targetDelta[ACTUATOR_COUNT] = {0, 0, 0, 0};
int16_t appliedDelta[ACTUATOR_COUNT] = {0, 0, 0, 0};
OutputDirection outputDirA[ACTUATOR_COUNT] = {DIR_IDLE, DIR_IDLE, DIR_IDLE, DIR_IDLE};
unsigned long lastStopA[ACTUATOR_COUNT] = {0, 0, 0, 0};

char commandLine[128];
size_t commandLength = 0;

volatile bool sr04EchoActive = false;
volatile bool sr04SampleReady = false;
volatile unsigned long sr04RiseUS = 0;
volatile unsigned long sr04PulseUS = 0;
unsigned long sr04TriggerUS = 0;
unsigned long sr04LastRequestMS = 0;
uint32_t sr04Sequence = 0;
int32_t sr04DistanceMM = -1;
char sr04Status[20] = "no_sample";

ISR(PCINT0_vect) {
  const bool high = (PINB & _BV(PB1)) != 0;  // D52 / PB1 / PCINT1
  const unsigned long now = micros();
  if (high) {
    sr04RiseUS = now;
    sr04EchoActive = true;
  } else if (sr04EchoActive) {
    sr04PulseUS = now - sr04RiseUS;
    sr04EchoActive = false;
    sr04SampleReady = true;
  }
}

void triggerSr04IfDue() {
  const unsigned long nowMS = millis();
  if (nowMS - sr04LastRequestMS < SR04_PERIOD_MS || sr04EchoActive) return;
  sr04LastRequestMS = nowMS;
  noInterrupts(); sr04SampleReady = false; sr04PulseUS = 0; interrupts();
  digitalWrite(SR04_TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(SR04_TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(SR04_TRIG_PIN, LOW);
  sr04TriggerUS = micros();
}

void updateSr04() {
  bool ready;
  unsigned long pulse;
  noInterrupts(); ready = sr04SampleReady; pulse = sr04PulseUS; if (ready) sr04SampleReady = false; interrupts();
  if (ready) {
    sr04TriggerUS = 0;
    int32_t mm = (int32_t)((pulse * 343UL + 1000UL) / 2000UL);
    sr04Sequence++;
    if (mm >= SR04_MIN_MM && mm <= SR04_MAX_MM) {
      sr04DistanceMM = mm; strncpy(sr04Status, "ok", sizeof(sr04Status));
    } else {
      sr04DistanceMM = -1; strncpy(sr04Status, "range_invalid", sizeof(sr04Status));
    }
  } else if (sr04TriggerUS != 0 && micros() - sr04TriggerUS > SR04_ECHO_TIMEOUT_US) {
    sr04TriggerUS = 0; sr04EchoActive = false; sr04Sequence++;
    sr04DistanceMM = -1; strncpy(sr04Status, "no_echo", sizeof(sr04Status));
  }
  triggerSr04IfDue();
}

void printSr04() {
  unsigned long pulse;
  noInterrupts(); pulse = sr04PulseUS; interrupts();
  Serial.print("SR04 seq="); Serial.print(sr04Sequence);
  Serial.print(" t_ms="); Serial.print(millis());
  Serial.print(" distance_mm="); Serial.print(sr04DistanceMM);
  Serial.print(" pulse_us="); Serial.print(pulse);
  Serial.print(" status="); Serial.println(sr04Status);
}

void stopLeg(uint8_t a) {
  digitalWrite(MOTOR_IN1[a], LOW);
  digitalWrite(MOTOR_IN2[a], LOW);
  digitalWrite(MOTOR_EN[a], LOW);
  outputDirA[a] = DIR_IDLE;
  lastStopA[a] = millis();
}
void hardStopAll() {
  for (uint8_t a = 0; a < ACTUATOR_COUNT; ++a) stopLeg(a);
}
void driveLeg(uint8_t a, OutputDirection direction) {
  digitalWrite(MOTOR_IN1[a], direction == DIR_EXTEND ? HIGH : LOW);
  digitalWrite(MOTOR_IN2[a], direction == DIR_RETRACT ? HIGH : LOW);
  digitalWrite(MOTOR_EN[a], HIGH);
  outputDirA[a] = direction;
}

bool selectTCAChannel(uint8_t channel) {
  if (channel > 7) return false;
  Wire.beginTransmission(TCA_ADDRESS);
  Wire.write((uint8_t)(1U << channel));
  if (Wire.endTransmission() != 0) return false;
  delayMicroseconds(300);
  return true;
}
bool disableAllTCAChannels() {
  Wire.beginTransmission(TCA_ADDRESS); Wire.write((uint8_t)0);
  return Wire.endTransmission() == 0;
}
bool initializeSensor(uint8_t a) {
  if (!selectTCAChannel(SENSOR_CHANNELS[a])) return false;
  delay(50); sensors[a].setTimeout(500);
  if (!sensors[a].init()) return false;
  sensors[a].setDistanceMode(VL53L1X::Short);
  if (!sensors[a].setMeasurementTimingBudget(50000)) return false;
  sensors[a].startContinuous(60);
  return true;
}
bool readSensor(uint8_t a) {
  bool valid = false; uint16_t distance = 0;
  if (selectTCAChannel(SENSOR_CHANNELS[a])) {
    distance = sensors[a].read();
    valid = !sensors[a].timeoutOccurred() &&
            sensors[a].ranging_data.range_status == VL53L1X::RangeValid &&
            distance > 0 && distance <= 4000;
  }
  if (valid) {
    currentMM[a] = distance;
    if (!filterInitialized[a]) {
      filteredMM[a] = (int16_t)distance;
      filterInitialized[a] = true;
    } else {
      filteredMM[a] = (int16_t)((3L * filteredMM[a] + distance + 2) / 4);
    }
    sensorValid[a] = true;
    lastValidMS[a] = millis(); measurementVersion[a]++; return true;
  }
  if (lastValidMS[a] == 0 || millis() - lastValidMS[a] > SENSOR_STALE_TIMEOUT_MS)
    sensorValid[a] = false;
  return false;
}
void updateOneSensor() {
  if (millis() - lastSensorReadMS < SENSOR_READ_GAP_MS) return;
  lastSensorReadMS = millis(); readSensor(nextSensor);
  nextSensor = (uint8_t)((nextSensor + 1) % ACTUATOR_COUNT);
}
bool allSensorsFreshAndInRange() {
  unsigned long now = millis();
  for (uint8_t a = 0; a < ACTUATOR_COUNT; ++a) {
    if (!sensorValid[a] || now - lastValidMS[a] > SENSOR_STALE_TIMEOUT_MS ||
        currentMM[a] < SENSOR_RANGE_MIN_MM || currentMM[a] > SENSOR_RANGE_MAX_MM) return false;
  }
  return true;
}

void enterDisarmed() {
  hardStopAll(); state = STATE_DISARMED; lastCommandSeq = 0;
  for (uint8_t i = 0; i < ACTUATOR_COUNT; ++i) {
    targetDelta[i] = 0; appliedDelta[i] = 0;
  }
}
void latchFault(const char *reason) {
  hardStopAll(); state = STATE_FAULT; faultCount++;
  strncpy(faultReason, reason, sizeof(faultReason) - 1);
  faultReason[sizeof(faultReason) - 1] = '\0';
  Serial.print("FAULT_LATCHED reason="); Serial.println(faultReason);
}
void enterEstop() {
  hardStopAll(); estopLatched = true; state = STATE_ESTOP;
  strncpy(faultReason, "estop", sizeof(faultReason));
  Serial.println("ESTOP_LATCHED ALL_OUTPUTS_LOW RESET_REQUIRED");
}

void updateAppliedDeltas() {
  if (state == STATE_BOOT || state == STATE_DISARMED || state == STATE_FAULT || state == STATE_ESTOP) {
    for (uint8_t i = 0; i < ACTUATOR_COUNT; ++i) appliedDelta[i] = 0;
    return;
  }
  for (uint8_t solver = 0; solver < ACTUATOR_COUNT; ++solver) {
    uint8_t a = SOLVER_TO_MEGA[solver];
    appliedDelta[solver] = (int16_t)currentMM[a] - armBaselineA[a];
  }
}

void updateControl() {
  if (state != STATE_ARMED && state != STATE_ACTIVE) return;
  unsigned long now = millis();
  if (now - lastHeartbeatMS > HEARTBEAT_TIMEOUT_MS) { latchFault("heartbeat_timeout"); return; }
  if (!allSensorsFreshAndInRange()) { latchFault("sensor_invalid_or_range"); return; }

  updateAppliedDeltas();
  bool anyMoving = false;
  for (uint8_t solver = 0; solver < ACTUATOR_COUNT; ++solver) {
    uint8_t a = SOLVER_TO_MEGA[solver];
    int16_t error = targetDelta[solver] - appliedDelta[solver];

    if (targetDelta[solver] == 0 || abs(error) <= TARGET_TOLERANCE_MM) {
      if (outputDirA[a] != DIR_IDLE) stopLeg(a);
      continue;
    }

    OutputDirection desired = error > 0 ? DIR_EXTEND : DIR_RETRACT;
    if (outputDirA[a] == desired) {
      anyMoving = true;  // continuous drive; do not pulse or stop between samples
      continue;
    }
    if (outputDirA[a] != DIR_IDLE) {
      stopLeg(a);
      continue;
    }
    if (now - lastStopA[a] < REVERSAL_DEAD_TIME_MS) continue;
    driveLeg(a, desired);
    anyMoving = true;
  }
  state = anyMoving ? STATE_ACTIVE : STATE_ARMED;
}

const char *stateName() {
  switch (state) {
    case STATE_BOOT: return "BOOT"; case STATE_DISARMED: return "DISARMED";
    case STATE_ARMED: return "ARMED"; case STATE_ACTIVE: return "ACTIVE";
    case STATE_FAULT: return "FAULT"; case STATE_ESTOP: return "ESTOP";
  }
  return "UNKNOWN";
}
const char *dirName(OutputDirection d) {
  return d == DIR_EXTEND ? "EXTEND" : d == DIR_RETRACT ? "RETRACT" : "IDLE";
}
void printStatus() {
  updateAppliedDeltas();
  Serial.print("IMU_STATUS mode="); Serial.print(stateName());
  Serial.print(" estop="); Serial.print(estopLatched ? 1 : 0);
  Serial.print(" faults="); Serial.print(faultCount);
  Serial.print(" fault="); Serial.print(faultReason);
  Serial.print(" seq="); Serial.print(lastCommandSeq);
  for (uint8_t a = 0; a < ACTUATOR_COUNT; ++a) {
    Serial.print(" A"); Serial.print(a + 1); Serial.print("="); Serial.print(currentMM[a]);
    Serial.print(","); Serial.print(sensorValid[a] ? 1 : 0);
    Serial.print(","); Serial.print(measurementVersion[a]);
    Serial.print(","); Serial.print(dirName(outputDirA[a]));
    Serial.print(","); Serial.print(directionVerified[a] ? 1 : 0);
  }
  Serial.print(" target=");
  for (uint8_t i = 0; i < 4; ++i) { if (i) Serial.print(","); Serial.print(targetDelta[i]); }
  Serial.print(" applied=");
  for (uint8_t i = 0; i < 4; ++i) { if (i) Serial.print(","); Serial.print(appliedDelta[i]); }
  Serial.println();
}

bool parseUint32(const char *token, uint32_t &value) {
  if (!token || !*token || *token == '-') return false;
  char *end = NULL; unsigned long parsed = strtoul(token, &end, 10);
  if (*end != '\0') return false; value = (uint32_t)parsed; return true;
}
bool parseInt16(const char *token, int16_t &value) {
  if (!token || !*token) return false;
  char *end = NULL; long parsed = strtol(token, &end, 10);
  if (*end != '\0' || parsed < -DELTA_PROTOCOL_MAX_MM || parsed > DELTA_PROTOCOL_MAX_MM) return false;
  value = (int16_t)parsed; return true;
}

void processCommand(char *command) {
  if (strcmp(command, "ESTOP") == 0) { enterEstop(); return; }
  if (strcmp(command, "STATUS") == 0) { printStatus(); return; }
  if (strcmp(command, "GET_DISTANCE") == 0) { printSr04(); return; }
  if (strcmp(command, "PING") == 0) { Serial.println("PONG MEGA_LANDER_IMU_V2"); return; }
  if (estopLatched || state == STATE_ESTOP) { Serial.println("ERR ESTOP_LATCHED"); return; }
  if (strcmp(command, "STOP") == 0) {
    bool recoverable = strcmp(faultReason, "heartbeat_timeout") == 0;
    if (state == STATE_FAULT && !recoverable) {
      Serial.println("ERR FAULT_LATCHED_RESET_REQUIRED"); return;
    }
    strncpy(faultReason, "none", sizeof(faultReason));
    enterDisarmed(); Serial.println("OK STOPPED DISARMED ALL_LOW"); return;
  }
  if (state == STATE_FAULT) { Serial.println("ERR FAULT_LATCHED_RESET_REQUIRED"); return; }

  if (strcmp(command, "ARM IMU") == 0) {
    if (!allSensorsFreshAndInRange()) { Serial.println("ERR ARM_SENSOR_GATE"); return; }
    hardStopAll();
    for (uint8_t a = 0; a < ACTUATOR_COUNT; ++a) {
      armBaselineA[a] = (int16_t)currentMM[a];
    }
    for (uint8_t i = 0; i < 4; ++i) { targetDelta[i] = 0; appliedDelta[i] = 0; }
    state = STATE_ARMED; lastHeartbeatMS = millis(); lastCommandSeq = 0;
    Serial.println("OK ARMED_IMU BASELINE_CAPTURED"); return;
  }

  char *save = NULL; char *kind = strtok_r(command, " ", &save);
  char *seqToken = strtok_r(NULL, " ", &save); uint32_t seq = 0;
  if (!parseUint32(seqToken, seq) || seq == 0 || seq <= lastCommandSeq) {
    Serial.println("ERR SEQ_MUST_INCREASE_NONZERO"); return;
  }
  if (state != STATE_ARMED && state != STATE_ACTIVE) { Serial.println("ERR NOT_ARMED"); return; }

  if (strcmp(kind, "HEARTBEAT") == 0) {
    if (strtok_r(NULL, " ", &save) != NULL) { Serial.println("ERR HEARTBEAT_FORMAT"); return; }
    lastCommandSeq = seq; lastHeartbeatMS = millis(); Serial.println("OK HEARTBEAT"); return;
  }
  if (strcmp(kind, "SET_DELTA") == 0) {
    int16_t candidate[4];
    for (uint8_t i = 0; i < 4; ++i) {
      if (!parseInt16(strtok_r(NULL, " ", &save), candidate[i])) {
        Serial.println("ERR SET_DELTA_FORMAT_OR_RANGE"); return;
      }
    }
    if (strtok_r(NULL, " ", &save) != NULL) { Serial.println("ERR SET_DELTA_EXTRA"); return; }
    for (uint8_t solver = 0; solver < ACTUATOR_COUNT; ++solver) {
      uint8_t a = SOLVER_TO_MEGA[solver];
      long absoluteTarget = (long)armBaselineA[a] + candidate[solver];
      if (absoluteTarget < SENSOR_RANGE_MIN_MM || absoluteTarget > SENSOR_RANGE_MAX_MM) {
        Serial.println("ERR SET_DELTA_ABSOLUTE_RANGE"); return;
      }
    }
    for (uint8_t i = 0; i < 4; ++i) targetDelta[i] = candidate[i];
    lastCommandSeq = seq; lastHeartbeatMS = millis();
    Serial.println("OK SET_DELTA"); return;
  }
  Serial.println("ERR UNKNOWN_COMMAND");
}

void pollCommands() {
  while (Serial.available()) {
    char ch = (char)Serial.read(); if (ch == '\r') continue;
    if (ch == '\n') {
      commandLine[commandLength] = '\0'; if (commandLength) processCommand(commandLine); commandLength = 0;
    } else if (commandLength < sizeof(commandLine) - 1) commandLine[commandLength++] = ch;
    else { commandLength = 0; latchFault("command_too_long"); }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(SR04_TRIG_PIN, OUTPUT); digitalWrite(SR04_TRIG_PIN, LOW);
  pinMode(SR04_ECHO_PIN, INPUT);
  for (uint8_t a = 0; a < 4; ++a) {
    pinMode(MOTOR_IN1[a], OUTPUT); pinMode(MOTOR_IN2[a], OUTPUT); pinMode(MOTOR_EN[a], OUTPUT);
  }
  hardStopAll(); delay(1500); Serial.println("MEGA_LANDER_IMU_V2 BOOT DISARMED ALL_LOW");
  Wire.begin(); Wire.setClock(100000); Wire.setWireTimeout(25000, true);
  if (!disableAllTCAChannels()) { latchFault("tca_not_detected"); return; }
  bool ok = true;
  for (uint8_t a = 0; a < 4; ++a) if (!initializeSensor(a)) ok = false;
  // VL53 continuous mode may need several measurement periods after init.
  // Keep every output LOW and wait a bounded 4 s for four fresh readings.
  unsigned long validationStartMS = millis();
  while (millis() - validationStartMS < 4000 && !allSensorsFreshAndInRange()) {
    hardStopAll();
    for (uint8_t a = 0; a < 4; ++a) readSensor(a);
    delay(20);
  }
  if (!ok) Serial.println("WARN SENSOR_INIT_RETRY_RECOVERED_IF_ALL_READINGS_VALID");
  if (!allSensorsFreshAndInRange()) { latchFault("sensor_init"); return; }
  // Enable ultrasonic Echo interrupts only after the safety-critical TCA/VL53
  // chain is fully initialized. A noisy or disconnected SR04 can never block it.
  PCICR |= _BV(PCIE0); PCMSK0 |= _BV(PCINT1);
  enterDisarmed(); Serial.println("READY IMU_V2 DISARMED");
}

void loop() {
  updateSr04();
  updateOneSensor();
  updateControl();
  pollCommands();
  if (state == STATE_DISARMED || state == STATE_FAULT || state == STATE_ESTOP) hardStopAll();
  if (millis() - lastStatusMS >= PERIODIC_STATUS_MS) { lastStatusMS = millis(); printStatus(); }
}
