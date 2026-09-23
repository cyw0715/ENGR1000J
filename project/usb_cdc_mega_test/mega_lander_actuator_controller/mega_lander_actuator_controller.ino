#include <Wire.h>
#include <VL53L1X.h>

// ==================================================
// TCA9548A 和 VL53L1X
// ==================================================

const uint8_t TCA_ADDRESS = 0x70;

const uint8_t SENSOR_COUNT = 4;
const uint8_t SENSOR_CHANNELS[SENSOR_COUNT] = {0, 1, 2, 3};

VL53L1X sensors[SENSOR_COUNT];

// ==================================================
// 四根推杆控制引脚
// ==================================================

const uint8_t MOTOR_IN1[4] = {22, 24, 26, 28};
const uint8_t MOTOR_IN2[4] = {23, 25, 27, 29};
const uint8_t MOTOR_EN[4]  = {30, 31, 32, 33};

// ==================================================
// 解算器长度 → 已验证 VL53L1X 距离坐标接口
// ==================================================
// sensor_target_mm = solver_length_mm - 190
// 解算器 250..390 mm 对应原程序传感器目标 60..200 mm。
const uint16_t SOLVER_MIN_MM = 250;
const uint16_t SOLVER_MAX_MM = 390;
const uint16_t SENSOR_MIN_MM = 60;
const uint16_t SENSOR_MAX_MM = 200;
const int16_t SOLVER_TO_SENSOR_OFFSET_MM = 190;
const uint8_t ACTUATOR_COUNT = 4;

// 每根推杆最长运行时间
const unsigned long MAX_MOVE_TIME_MS = 30000UL;

// 两次读取之间的间隔
const unsigned long READ_INTERVAL_MS = 60;

// 连续达到目标次数
const uint8_t REQUIRED_CONFIRMATIONS = 3;

// 运动过程中允许的连续无效读数次数
const uint8_t MAX_CONSECUTIVE_FAILURES = 10;

// 获取初始有效读数时的最大尝试次数
const uint8_t INITIAL_READ_ATTEMPTS = 20;

// ==================================================
// TCA9548A
// ==================================================

bool selectTCAChannel(uint8_t channel)
{
  if (channel > 7) {
    return false;
  }

  Wire.beginTransmission(TCA_ADDRESS);
  Wire.write((uint8_t)(1U << channel));

  uint8_t result = Wire.endTransmission();

  if (result != 0) {
    return false;
  }

  // 给模拟开关一点稳定时间
  delayMicroseconds(300);

  return true;
}

bool disableAllTCAChannels()
{
  Wire.beginTransmission(TCA_ADDRESS);
  Wire.write((uint8_t)0x00);

  return Wire.endTransmission() == 0;
}

// ==================================================
// 推杆控制
// ==================================================

void stopActuator(uint8_t actuatorIndex)
{
  digitalWrite(MOTOR_IN1[actuatorIndex], LOW);
  digitalWrite(MOTOR_IN2[actuatorIndex], LOW);
}

void stopAll()
{
  for (uint8_t i = 0; i < 4; i++) {
    stopActuator(i);
  }
}

// 假设 HIGH/LOW 对应伸长
void extendActuator(uint8_t actuatorIndex)
{
  stopActuator(actuatorIndex);
  delay(100);

  digitalWrite(MOTOR_IN1[actuatorIndex], HIGH);
  digitalWrite(MOTOR_IN2[actuatorIndex], LOW);
}

// 假设 LOW/HIGH 对应收缩
void retractActuator(uint8_t actuatorIndex)
{
  stopActuator(actuatorIndex);
  delay(100);

  digitalWrite(MOTOR_IN1[actuatorIndex], LOW);
  digitalWrite(MOTOR_IN2[actuatorIndex], HIGH);
}

// ==================================================
// 传感器初始化
// ==================================================

bool initializeSensor(uint8_t sensorIndex)
{
  uint8_t channel = SENSOR_CHANNELS[sensorIndex];

  Serial.print("Initializing sensor ");
  Serial.print(sensorIndex + 1);
  Serial.print(" on TCA channel ");
  Serial.print(channel);
  Serial.println("...");

  if (!selectTCAChannel(channel))
  {
    Serial.println("ERROR: Failed to select TCA channel.");
    return false;
  }

  delay(50);

  sensors[sensorIndex].setTimeout(500);

  if (!sensors[sensorIndex].init())
  {
    Serial.println("ERROR: VL53L1X initialization failed.");
    return false;
  }

  // 目标距离只有50～100 mm，使用短距离模式
  sensors[sensorIndex].setDistanceMode(VL53L1X::Short);

  // 每次测量允许50 ms
  if (!sensors[sensorIndex].setMeasurementTimingBudget(50000))
  {
    Serial.println("ERROR: Failed to set timing budget.");
    return false;
  }

  // 测量周期60 ms
  sensors[sensorIndex].startContinuous(60);

  Serial.println("Sensor initialized successfully.");

  return true;
}

// ==================================================
// 读取传感器
// ==================================================

bool readOneMeasurement(
  uint8_t sensorIndex,
  uint16_t &distanceMM,
  bool printInvalid
)
{
  if (!selectTCAChannel(SENSOR_CHANNELS[sensorIndex]))
  {
    if (printInvalid) {
      Serial.println("TCA channel selection failed.");
    }

    return false;
  }

  // 默认阻塞等待新测量完成
  distanceMM = sensors[sensorIndex].read();

  if (sensors[sensorIndex].timeoutOccurred())
  {
    if (printInvalid) {
      Serial.println("Sensor read timeout.");
    }

    return false;
  }

  VL53L1X::RangeStatus status =
    sensors[sensorIndex].ranging_data.range_status;

  if (status != VL53L1X::RangeValid)
  {
    if (printInvalid)
    {
      Serial.print("Invalid range status: ");
      Serial.print((uint8_t)status);

      Serial.print(", raw distance: ");
      Serial.print(distanceMM);
      Serial.println(" mm");
    }

    return false;
  }

  if (distanceMM == 0 || distanceMM > 4000)
  {
    if (printInvalid)
    {
      Serial.print("Invalid distance: ");
      Serial.print(distanceMM);
      Serial.println(" mm");
    }

    return false;
  }

  return true;
}

// 多次尝试，直到得到一个有效读数
bool readValidDistance(
  uint8_t sensorIndex,
  uint16_t &distanceMM,
  uint8_t maxAttempts
)
{
  for (uint8_t attempt = 1; attempt <= maxAttempts; attempt++)
  {
    if (readOneMeasurement(sensorIndex, distanceMM, true))
    {
      return true;
    }

    Serial.print("Sensor ");
    Serial.print(sensorIndex + 1);
    Serial.print(" retry ");
    Serial.print(attempt);
    Serial.print("/");
    Serial.println(maxAttempts);

    delay(READ_INTERVAL_MS);
  }

  return false;
}

// ==================================================
// 传感器预热
// ==================================================

bool warmUpSensors()
{
  Serial.println();
  Serial.println("Warming up sensors...");

  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
  {
    uint16_t distanceMM = 0;

    Serial.print("Waiting for valid reading from sensor ");
    Serial.println(i + 1);

    if (!readValidDistance(i, distanceMM, INITIAL_READ_ATTEMPTS))
    {
      Serial.print("ERROR: Sensor ");
      Serial.print(i + 1);
      Serial.println(" did not produce a valid reading.");

      return false;
    }

    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.print(" ready. Initial distance: ");
    Serial.print(distanceMM);
    Serial.println(" mm");
  }

  return true;
}

// ==================================================
// 移动一根推杆到目标
// ==================================================

bool moveActuatorToTarget(
  uint8_t actuatorIndex,
  uint16_t targetMM,
  bool distanceShouldIncrease
)
{
  uint8_t sensorIndex = actuatorIndex;

  stopAll();
  delay(200);

  uint16_t distanceMM = 0;

  // 初始读数不再只读一次
  if (!readValidDistance(
        sensorIndex,
        distanceMM,
        INITIAL_READ_ATTEMPTS
      ))
  {
    Serial.print("ERROR: Cannot obtain a valid initial reading from sensor ");
    Serial.println(sensorIndex + 1);

    return false;
  }

  Serial.println();
  Serial.print("Actuator ");
  Serial.print(actuatorIndex + 1);
  Serial.print(" initial distance: ");
  Serial.print(distanceMM);
  Serial.println(" mm");

  if (distanceShouldIncrease)
  {
    if (distanceMM >= targetMM)
    {
      Serial.println("Extension target already reached.");
      return true;
    }

    Serial.print("Extending actuator ");
    Serial.print(actuatorIndex + 1);
    Serial.print(" toward ");
    Serial.print(targetMM);
    Serial.println(" mm");

    extendActuator(actuatorIndex);
  }
  else
  {
    if (distanceMM <= targetMM)
    {
      Serial.println("Retraction target already reached.");
      return true;
    }

    Serial.print("Retracting actuator ");
    Serial.print(actuatorIndex + 1);
    Serial.print(" toward ");
    Serial.print(targetMM);
    Serial.println(" mm");

    retractActuator(actuatorIndex);
  }

  unsigned long startTime = millis();

  uint8_t confirmationCount = 0;
  uint8_t failureCount = 0;

  while (true)
  {
    if (millis() - startTime >= MAX_MOVE_TIME_MS)
    {
      stopActuator(actuatorIndex);

      Serial.print("ERROR: Actuator ");
      Serial.print(actuatorIndex + 1);
      Serial.println(" movement timed out.");

      return false;
    }

    if (!readOneMeasurement(sensorIndex, distanceMM, true))
    {
      failureCount++;

      Serial.print("Consecutive sensor failures: ");
      Serial.println(failureCount);

      if (failureCount >= MAX_CONSECUTIVE_FAILURES)
      {
        stopActuator(actuatorIndex);

        Serial.print("ERROR: Sensor ");
        Serial.print(sensorIndex + 1);
        Serial.println(" failed repeatedly.");

        return false;
      }

      delay(READ_INTERVAL_MS);
      continue;
    }

    failureCount = 0;

    Serial.print("Actuator ");
    Serial.print(actuatorIndex + 1);
    Serial.print(" | Distance: ");
    Serial.print(distanceMM);
    Serial.println(" mm");

    bool reached;

    if (distanceShouldIncrease) {
      reached = distanceMM >= targetMM;
    } else {
      reached = distanceMM <= targetMM;
    }

    if (reached) {
      confirmationCount++;
    } else {
      confirmationCount = 0;
    }

    if (confirmationCount >= REQUIRED_CONFIRMATIONS)
    {
      stopActuator(actuatorIndex);

      Serial.print("Actuator ");
      Serial.print(actuatorIndex + 1);
      Serial.print(" reached target. Final distance: ");
      Serial.print(distanceMM);
      Serial.println(" mm");

      delay(300);
      return true;
    }

    delay(READ_INTERVAL_MS);
  }
}

// ==================================================
// 解算器数据接口（唯一新增控制入口）
// P4 USB Host 发送 LF 结尾的一行：
//   SET_TARGET <FL> <FR> <RL> <RR>
// 四个输入均是解算器轴向长度，范围 250..390 mm。
// ==================================================

char commandLine[96];
size_t commandLength = 0;

bool solverLengthToSensorTarget(long solverMM, uint16_t &sensorTargetMM)
{
  if (solverMM < SOLVER_MIN_MM || solverMM > SOLVER_MAX_MM) {
    return false;
  }

  long converted = solverMM - SOLVER_TO_SENSOR_OFFSET_MM;
  if (converted < SENSOR_MIN_MM || converted > SENSOR_MAX_MM) {
    return false;
  }

  sensorTargetMM = (uint16_t)converted;
  return true;
}

bool parseSolverTargets(char *arguments, uint16_t sensorTargets[ACTUATOR_COUNT])
{
  for (uint8_t i = 0; i < ACTUATOR_COUNT; i++)
  {
    char *token = (i == 0) ? strtok(arguments, " ") : strtok(NULL, " ");
    char *end = NULL;
    if (token == NULL) {
      Serial.println("ERR set_target_requires_FL_FR_RL_RR");
      return false;
    }

    long solverMM = strtol(token, &end, 10);
    if (*token == '\0' || *end != '\0' ||
        !solverLengthToSensorTarget(solverMM, sensorTargets[i]))
    {
      Serial.println("ERR solver_targets_must_be_250_to_390_mm");
      return false;
    }
  }

  if (strtok(NULL, " ") != NULL) {
    Serial.println("ERR set_target_requires_exactly_four_values");
    return false;
  }

  return true;
}

bool moveSolverTarget(uint8_t actuatorIndex, uint16_t sensorTargetMM)
{
  // 只为选择原程序已有的伸/缩方向而先读一次；真正动作仍由原有
  // moveActuatorToTarget() 完成，其中保留初始复读、3 次确认、30 秒
  // 超时与连续无效读数保护。
  uint16_t currentMM = 0;
  if (!readValidDistance(actuatorIndex, currentMM, INITIAL_READ_ATTEMPTS)) {
    Serial.print("ERROR: Cannot select direction for actuator ");
    Serial.println(actuatorIndex + 1);
    return false;
  }

  return moveActuatorToTarget(
    actuatorIndex,
    sensorTargetMM,
    currentMM < sensorTargetMM
  );
}

void executeSolverTargets(uint16_t sensorTargets[ACTUATOR_COUNT])
{
  Serial.print("TARGET_ACCEPTED sensor_mm=");
  for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
    Serial.print(sensorTargets[i]);
    if (i < ACTUATOR_COUNT - 1) Serial.print(",");
  }
  Serial.println();

  // 沿用原测试程序的顺序：一次只移动一条推杆。任一原有保护失败时，
  // 进入原 emergencyStop() 的永久停机状态。
  for (uint8_t i = 0; i < ACTUATOR_COUNT; i++)
  {
    if (!moveSolverTarget(i, sensorTargets[i])) {
      emergencyStop("Solver target sequence failed.");
    }
  }

  stopAll();
  Serial.println("TARGET_SEQUENCE_COMPLETED");
}

void processCommand(char *command)
{
  if (strcmp(command, "PING") == 0) {
    Serial.println("PONG");
    return;
  }

  if (strncmp(command, "SET_TARGET ", 11) == 0) {
    uint16_t sensorTargets[ACTUATOR_COUNT];
    if (parseSolverTargets(command + 11, sensorTargets)) {
      executeSolverTargets(sensorTargets);
    }
    return;
  }

  // 旧 P4 固件可能仍会发出超声波轮询命令。明确忽略，绝不触发动作。
  if (strcmp(command, "GET_DISTANCE") == 0) {
    return;
  }

  Serial.println("ERR unknown_command");
}

void pollCommands()
{
  while (Serial.available() > 0)
  {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;

    if (ch == '\n') {
      commandLine[commandLength] = '\0';
      if (commandLength > 0) {
        processCommand(commandLine);
      }
      commandLength = 0;
    } else if (commandLength < sizeof(commandLine) - 1) {
      commandLine[commandLength++] = ch;
    } else {
      commandLength = 0;
      Serial.println("ERR command_too_long");
    }
  }
}

// ==================================================
// 紧急停止
// ==================================================

void emergencyStop(const char *message)
{
  stopAll();

  Serial.println();
  Serial.println("EMERGENCY STOP");
  Serial.println(message);
  Serial.println("All actuators stopped.");

  while (true)
  {
    stopAll();
    delay(1000);
  }
}

// ==================================================
// setup
// ==================================================

void setup()
{
  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println("System starting...");

  for (uint8_t i = 0; i < 4; i++)
  {
    pinMode(MOTOR_IN1[i], OUTPUT);
    pinMode(MOTOR_IN2[i], OUTPUT);
    pinMode(MOTOR_EN[i], OUTPUT);

    digitalWrite(MOTOR_EN[i], HIGH);
  }

  stopAll();

  Wire.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(25000, true);

  Serial.println("I2C started.");

  if (!disableAllTCAChannels())
  {
    emergencyStop("TCA9548A not detected.");
  }

  delay(100);

  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
  {
    if (!initializeSensor(i))
    {
      emergencyStop("VL53L1X initialization failed.");
    }
  }

  // 等待连续测量开始
  delay(300);

  if (!warmUpSensors())
  {
    emergencyStop("Sensor warm-up failed.");
  }

  Serial.println();
  Serial.println("All sensors initialized and producing valid readings.");
  Serial.println("MEGA_LANDER_READY solver_range_mm=250..390 sensor_range_mm=60..200 map=sensor=solver-190");
  Serial.println("Waiting for SET_TARGET FL FR RL RR; no automatic movement.");
}

void loop()
{
  pollCommands();
}