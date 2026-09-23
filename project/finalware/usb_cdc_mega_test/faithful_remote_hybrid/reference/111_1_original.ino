#include <Wire.h>
#include <VL53L1X.h>
#include <PS2X_lib.h>

// ==================================================
// PS2 接收器引脚
// ==================================================

const uint8_t PS2_DAT = 49;
const uint8_t PS2_CMD = 47;
const uint8_t PS2_CLK = 46;
const uint8_t PS2_CS  = 48;

PS2X ps2x;

int errorCode = 1;
bool ps2Connected = false;

unsigned long lastReconnectAttempt = 0;
const unsigned long RECONNECT_INTERVAL_MS = 2000;

// 连续读取失败达到该次数后，认定手柄已经断开。
// 这样可以避免一次偶发通信错误导致立刻退出。
const uint8_t PS2_MAX_READ_FAILURES = 3;
uint8_t ps2ReadFailureCount = 0;

// 每次手柄连接/重新连接后，必须先检测到START已经松开，
// 才允许START切换模式。这样可以防止连接时按住START，
// 或PS2X库保留旧按键状态，导致MANUAL立刻又切到PRESET。
bool startToggleArmed = false;

// START按键的上一次状态由Mega保存。
// 模式切换依赖主板检测“松开 -> 按下”的变化，
// 不使用PS2X库内部保存的ButtonPressed状态。
bool previousStartButton = false;

// 进入预设模式时的震动提示。
// 震动只做提示，不参与决定当前模式。
const unsigned long PRESET_MODE_RUMBLE_DURATION_MS = 450;
const uint8_t PRESET_MODE_RUMBLE_STRENGTH = 255;

bool modeRumbleActive = false;
unsigned long modeRumbleStartTime = 0;


// ==================================================
// TCA9548A 和 VL53L1X
// ==================================================

const uint8_t TCA_ADDRESS = 0x70;
const uint8_t SENSOR_COUNT = 4;

// 推杆1、2、3、4分别对应TCA通道0、1、2、3
const uint8_t SENSOR_CHANNELS[SENSOR_COUNT] = {0, 1, 2, 3};

VL53L1X sensors[SENSOR_COUNT];

uint16_t currentLengthMM[SENSOR_COUNT] = {0, 0, 0, 0};
bool sensorValid[SENSOR_COUNT] = {false, false, false, false};
unsigned long lastValidSensorTime[SENSOR_COUNT] = {0, 0, 0, 0};

// 单次无效读数不会立刻判定传感器失效。
// 只有超过该时间没有得到有效读数，才认为传感器失效。
const unsigned long SENSOR_STALE_TIMEOUT_MS = 1200;

// 每次产生有效测量后加1
uint32_t measurementVersion[SENSOR_COUNT] = {0, 0, 0, 0};

uint8_t nextSensorIndex = 0;

unsigned long lastSensorReadTime = 0;
unsigned long lastSerialPrintTime = 0;

const unsigned long SENSOR_READ_GAP_MS = 10;
const unsigned long SERIAL_PRINT_INTERVAL_MS = 500;


// ==================================================
// 四根推杆的 L298N 控制引脚
//
// 推杆1：第一块L298N OUT1/OUT2
// 推杆2：第一块L298N OUT3/OUT4
// 推杆3：第二块L298N OUT1/OUT2
// 推杆4：第二块L298N OUT3/OUT4
// ==================================================

const uint8_t MOTOR_IN1[4] = {22, 24, 26, 28};
const uint8_t MOTOR_IN2[4] = {23, 25, 27, 29};
const uint8_t MOTOR_EN[4]  = {30, 31, 32, 33};


// ==================================================
// 16组预设长度
//
// 每一行代表一个预设位置，单位为 mm。
// 每行四个数值的顺序固定为：
// {推杆1, 推杆2, 推杆3, 推杆4}
//
// 下面先沿用你原来的目标值作为安全占位值。
// 请根据实际需要逐行修改16组数据。
// ==================================================

const uint8_t PRESET_COUNT = 16;

const uint16_t PRESET_LENGTH_MM[PRESET_COUNT][4] = {
  {164, 101, 147, 138},  // 预设位置0：L2 D
  {89, 102, 146, 138},  // 预设位置1：L1 D
  {200, 80, 176, 177},  // 预设位置2：左侧上键 D
  {158, 84, 183, 174},  // 预设位置3：左侧左键 D
  {161, 96, 87, 140},  // 预设位置4：左侧下键 D
  {99, 100, 82, 139},  // 预设位置5：左侧右键 D
  {222, 93, 147, 190},  // 预设位置6：左摇杆向左
  {159, 87, 135, 185},  // 预设位置7：左摇杆向右 D
  {168, 107, 147, 83},  // 预设位置8：右摇杆向左 D
  {98, 106, 152, 82},  // 预设位置9：右摇杆向右 D
  {229, 92, 204, 134},  // 预设位置10：右侧左键（方形）D
  {166, 94, 213, 143},  // 预设位置11：右侧下键（叉形) D
  {161, 96, 92, 82},  // 预设位置12：右侧右键（圆形）D
  {110, 108, 96, 102},  // 预设位置13：右侧上键（三角形）D
  {229, 96, 152, 134},  // 预设位置14：R1 D
  {162, 94, 136, 125}   // 预设位置15：R2 D
};

// 允许目标误差，例如目标150 mm，147～153 mm都认为到达
const uint16_t TARGET_TOLERANCE_MM = 3;

// 连续两次测量都在目标范围内，才认为真正到达
const uint8_t REQUIRED_CONFIRMATIONS = 2;

// 自动调整最长允许时间
const unsigned long PRESET_TIMEOUT_MS = 30000;


// ==================================================
// 主控制模式
// ==================================================

enum ControlMode : uint8_t
{
  MODE_MANUAL,
  MODE_PRESET_SELECT
};

ControlMode controlMode = MODE_MANUAL;


// ==================================================
// 自动移动到预设位置的状态
// ==================================================

bool presetMovementActive = false;
uint8_t activePresetIndex = 0;

bool presetReached[4] = {
  false, false, false, false
};

uint8_t presetConfirmationCount[4] = {
  0, 0, 0, 0
};

uint32_t presetProcessedVersion[4] = {
  0, 0, 0, 0
};

unsigned long presetStartTime = 0;


// ==================================================
// 预设选择按键锁
//
// 预设模式中使用 Button() 读取按住状态，再用这个锁保证
// 每次按下只触发一次。必须松开按键/摇杆回中后才能再次选择。
// ==================================================

const uint8_t STICK_LEFT_THRESHOLD = 70;
const uint8_t STICK_RIGHT_THRESHOLD = 185;
const uint8_t STICK_CENTER_LOW = 100;
const uint8_t STICK_CENTER_HIGH = 155;

bool presetSelectionLatched = false;


// ==================================================
// TCA9548A通道选择
// ==================================================

bool selectTCAChannel(uint8_t channel)
{
  if (channel > 7)
  {
    return false;
  }

  Wire.beginTransmission(TCA_ADDRESS);
  Wire.write((uint8_t)(1U << channel));

  uint8_t result = Wire.endTransmission();

  if (result != 0)
  {
    return false;
  }

  delayMicroseconds(300);

  return true;
}


// ==================================================
// 推杆控制函数
// ==================================================

void extendActuator(uint8_t actuatorIndex)
{
  digitalWrite(MOTOR_EN[actuatorIndex], HIGH);

  /*
    第一输出端为正：
    OUT1或OUT3为正，OUT2或OUT4为负。
  */
  digitalWrite(MOTOR_IN1[actuatorIndex], HIGH);
  digitalWrite(MOTOR_IN2[actuatorIndex], LOW);
}

void retractActuator(uint8_t actuatorIndex)
{
  digitalWrite(MOTOR_EN[actuatorIndex], HIGH);

  // 反转输出极性
  digitalWrite(MOTOR_IN1[actuatorIndex], LOW);
  digitalWrite(MOTOR_IN2[actuatorIndex], HIGH);
}

void stopActuator(uint8_t actuatorIndex)
{
  digitalWrite(MOTOR_IN1[actuatorIndex], LOW);
  digitalWrite(MOTOR_IN2[actuatorIndex], LOW);
}

void stopAllActuators()
{
  for (uint8_t i = 0; i < 4; i++)
  {
    stopActuator(i);
  }
}


// ==================================================
// 手动控制一根推杆
// ==================================================

void controlActuator(
  uint8_t actuatorIndex,
  bool extendPressed,
  bool retractPressed
)
{
  if (extendPressed && !retractPressed)
  {
    extendActuator(actuatorIndex);
  }
  else if (retractPressed && !extendPressed)
  {
    retractActuator(actuatorIndex);
  }
  else
  {
    stopActuator(actuatorIndex);
  }
}


// ==================================================
// 初始化一个VL53L1X
// ==================================================

bool initializeSensor(uint8_t sensorIndex)
{
  if (!selectTCAChannel(SENSOR_CHANNELS[sensorIndex]))
  {
    Serial.print("ERROR: Cannot select TCA channel ");
    Serial.println(SENSOR_CHANNELS[sensorIndex]);

    return false;
  }

  delay(50);

  sensors[sensorIndex].setTimeout(500);

  if (!sensors[sensorIndex].init())
  {
    Serial.print("ERROR: Sensor ");
    Serial.print(sensorIndex + 1);
    Serial.println(" initialization failed.");

    return false;
  }

  // 当前测量距离较短，使用短距离模式
  sensors[sensorIndex].setDistanceMode(VL53L1X::Short);

  // 每次测量预算50 ms
  if (!sensors[sensorIndex].setMeasurementTimingBudget(50000))
  {
    Serial.print("ERROR: Sensor ");
    Serial.print(sensorIndex + 1);
    Serial.println(" timing budget failed.");

    return false;
  }

  // 每60 ms产生一次测量
  sensors[sensorIndex].startContinuous(60);

  Serial.print("Sensor ");
  Serial.print(sensorIndex + 1);
  Serial.print(" initialized on TCA channel ");
  Serial.println(SENSOR_CHANNELS[sensorIndex]);

  return true;
}


// ==================================================
// 读取一个VL53L1X
// ==================================================

bool readSensor(uint8_t sensorIndex)
{
  bool validReading = false;
  uint16_t distanceMM = 0;

  if (selectTCAChannel(SENSOR_CHANNELS[sensorIndex]))
  {
    distanceMM = sensors[sensorIndex].read();

    if (!sensors[sensorIndex].timeoutOccurred())
    {
      VL53L1X::RangeStatus status =
        sensors[sensorIndex].ranging_data.range_status;

      if (status == VL53L1X::RangeValid &&
          distanceMM > 0 &&
          distanceMM <= 4000)
      {
        validReading = true;
      }
    }
  }

  if (validReading)
  {
    currentLengthMM[sensorIndex] = distanceMM;
    sensorValid[sensorIndex] = true;
    lastValidSensorTime[sensorIndex] = millis();
    measurementVersion[sensorIndex]++;
    return true;
  }

  // 不因一次偶发无效读数立刻禁止预设模式。
  // 只有持续一段时间没有有效数据才判定失效。
  if (lastValidSensorTime[sensorIndex] == 0 ||
      millis() - lastValidSensorTime[sensorIndex] > SENSOR_STALE_TIMEOUT_MS)
  {
    sensorValid[sensorIndex] = false;
  }

  return false;
}


// ==================================================
// 轮流读取四个传感器
//
// 每次loop只读取一个传感器，避免一次读取四个传感器
// 导致手柄响应过慢。
// ==================================================

void updateOneSensor()
{
  if (millis() - lastSensorReadTime < SENSOR_READ_GAP_MS)
  {
    return;
  }

  lastSensorReadTime = millis();

  readSensor(nextSensorIndex);

  nextSensorIndex++;

  if (nextSensorIndex >= SENSOR_COUNT)
  {
    nextSensorIndex = 0;
  }
}


// ==================================================
// 串口显示四根推杆长度和当前模式
// ==================================================

void printAllLengths()
{
  if (millis() - lastSerialPrintTime < SERIAL_PRINT_INTERVAL_MS)
  {
    return;
  }

  lastSerialPrintTime = millis();

  for (uint8_t i = 0; i < 4; i++)
  {
    Serial.print("Actuator ");
    Serial.print(i + 1);
    Serial.print(": ");

    if (sensorValid[i])
    {
      Serial.print(currentLengthMM[i]);
      Serial.print(" mm");
    }
    else
    {
      Serial.print("INVALID");
    }

    if (i < 3)
    {
      Serial.print(" | ");
    }
  }

  Serial.print(" | Mode: ");

  if (controlMode == MODE_MANUAL)
  {
    Serial.println("MANUAL");
  }
  else if (presetMovementActive)
  {
    Serial.print("PRESET ");
    Serial.print(activePresetIndex + 1);
    Serial.println(" MOVING");
  }
  else
  {
    Serial.println("PRESET SELECT");
  }
}


// ==================================================
// 检查四个传感器是否都有有效数据
// ==================================================

bool allSensorsValid()
{
  for (uint8_t i = 0; i < 4; i++)
  {
    if (!sensorValid[i])
    {
      return false;
    }
  }

  return true;
}


// ==================================================
// 模式震动提示
// ==================================================

void startPresetModeRumble()
{
  modeRumbleActive = true;
  modeRumbleStartTime = millis();
}

uint8_t getCurrentRumbleStrength()
{
  if (!modeRumbleActive)
  {
    return 0;
  }

  if (millis() - modeRumbleStartTime >= PRESET_MODE_RUMBLE_DURATION_MS)
  {
    modeRumbleActive = false;
    return 0;
  }

  return PRESET_MODE_RUMBLE_STRENGTH;
}

void stopModeRumble()
{
  modeRumbleActive = false;
}


// ==================================================
// 进入预设位置选择模式
// ==================================================

void enterPresetSelectionMode()
{
  stopAllActuators();
  presetMovementActive = false;
  controlMode = MODE_PRESET_SELECT;

  // 模式由Mega上的controlMode决定。
  // 只有从手动模式进入预设模式时，主板发送一次短震动。
  startPresetModeRumble();

  // 进入预设模式时清除旧的选择锁。
  // 后续每次按键都必须经过“按下 -> 松开”才能再次触发。
  presetSelectionLatched = false;

  Serial.println();
  Serial.println("Entered PRESET SELECT mode.");
  Serial.println("Press a mapped button to choose preset 1-16.");
  Serial.println("Press START again to return to MANUAL mode.");
}


// ==================================================
// 返回手动控制模式
// ==================================================

void enterManualMode()
{
  stopAllActuators();
  presetMovementActive = false;
  controlMode = MODE_MANUAL;

  // 从预设模式返回手动模式时不震动，并立即关闭任何剩余震动。
  stopModeRumble();

  Serial.println();
  Serial.println("Returned to MANUAL mode.");
}


// ==================================================
// 开始移动到指定预设位置
// presetIndex使用0～15，对应预设位置1～16
// ==================================================

void startPresetMovement(uint8_t presetIndex)
{
  if (presetIndex >= PRESET_COUNT)
  {
    return;
  }

  if (!allSensorsValid())
  {
    Serial.println();
    Serial.println("Cannot start preset movement.");
    Serial.println("One or more sensor readings are invalid.");

    stopAllActuators();
    presetMovementActive = false;
    return;
  }

  stopAllActuators();

  activePresetIndex = presetIndex;
  presetMovementActive = true;
  presetStartTime = millis();

  for (uint8_t i = 0; i < 4; i++)
  {
    presetReached[i] = false;
    presetConfirmationCount[i] = 0;
    presetProcessedVersion[i] = measurementVersion[i];
  }

  Serial.println();
  Serial.print("Preset position ");
  Serial.print(activePresetIndex + 1);
  Serial.println(" selected.");

  for (uint8_t i = 0; i < 4; i++)
  {
    Serial.print("Actuator ");
    Serial.print(i + 1);
    Serial.print(" target: ");
    Serial.print(PRESET_LENGTH_MM[activePresetIndex][i]);
    Serial.println(" mm");
  }
}


// ==================================================
// 自动控制四根推杆达到当前预设位置
//
// 假设：推杆伸长时，传感器读数增加。
// ==================================================

void updatePresetMovement()
{
  if (!presetMovementActive)
  {
    return;
  }

  // 超时保护
  if (millis() - presetStartTime >= PRESET_TIMEOUT_MS)
  {
    stopAllActuators();
    presetMovementActive = false;

    Serial.println();
    Serial.print("ERROR: Preset position ");
    Serial.print(activePresetIndex + 1);
    Serial.println(" movement timed out.");
    Serial.println("All actuators stopped.");

    return;
  }

  // 自动运动中任何传感器失效，都停止全部推杆
  if (!allSensorsValid())
  {
    stopAllActuators();
    presetMovementActive = false;

    Serial.println();
    Serial.println("ERROR: Invalid sensor reading during preset movement.");
    Serial.println("All actuators stopped.");

    return;
  }

  bool allReached = true;

  for (uint8_t i = 0; i < 4; i++)
  {
    int targetLength =
      (int)PRESET_LENGTH_MM[activePresetIndex][i];

    int difference =
      targetLength - (int)currentLengthMM[i];

    int absoluteDifference = abs(difference);

    // 已进入目标误差范围
    if (absoluteDifference <= TARGET_TOLERANCE_MM)
    {
      stopActuator(i);

      /*
        只有出现新的测量结果时，才增加确认次数。
        防止同一个测量值被重复计算。
      */
      if (measurementVersion[i] != presetProcessedVersion[i])
      {
        presetProcessedVersion[i] = measurementVersion[i];
        presetConfirmationCount[i]++;
      }

      if (presetConfirmationCount[i] >= REQUIRED_CONFIRMATIONS)
      {
        presetReached[i] = true;
      }
      else
      {
        allReached = false;
      }
    }
    else
    {
      presetReached[i] = false;
      presetConfirmationCount[i] = 0;
      presetProcessedVersion[i] = measurementVersion[i];

      allReached = false;

      // 当前长度小于目标：伸长
      if (difference > 0)
      {
        extendActuator(i);
      }
      // 当前长度大于目标：收缩
      else
      {
        retractActuator(i);
      }
    }
  }

  if (allReached)
  {
    stopAllActuators();
    presetMovementActive = false;

    Serial.println();
    Serial.print("Preset position ");
    Serial.print(activePresetIndex + 1);
    Serial.println(" reached successfully.");

    for (uint8_t i = 0; i < 4; i++)
    {
      Serial.print("Actuator ");
      Serial.print(i + 1);
      Serial.print(" final length: ");
      Serial.print(currentLengthMM[i]);
      Serial.println(" mm");
    }

    Serial.println("Still in PRESET SELECT mode.");
    Serial.println("Choose another preset or press START for MANUAL mode.");
  }
}


// ==================================================
// 检测预设位置选择
//
// 返回值：
// -1 代表没有选择
// 0～15代表预设位置1～16
// ==================================================

int8_t detectPresetSelection()
{
  int leftX = ps2x.Analog(PSS_LX);
  int rightX = ps2x.Analog(PSS_RX);

  bool leftStickLeft = leftX < STICK_LEFT_THRESHOLD;
  bool leftStickRight = leftX > STICK_RIGHT_THRESHOLD;
  bool rightStickLeft = rightX < STICK_LEFT_THRESHOLD;
  bool rightStickRight = rightX > STICK_RIGHT_THRESHOLD;

  bool anyPresetInput =
    ps2x.Button(PSB_L2) ||
    ps2x.Button(PSB_L1) ||
    ps2x.Button(PSB_PAD_UP) ||
    ps2x.Button(PSB_PAD_LEFT) ||
    ps2x.Button(PSB_PAD_DOWN) ||
    ps2x.Button(PSB_PAD_RIGHT) ||
    leftStickLeft ||
    leftStickRight ||
    rightStickLeft ||
    rightStickRight ||
    ps2x.Button(PSB_SQUARE) ||
    ps2x.Button(PSB_CROSS) ||
    ps2x.Button(PSB_CIRCLE) ||
    ps2x.Button(PSB_TRIANGLE) ||
    ps2x.Button(PSB_R1) ||
    ps2x.Button(PSB_R2);

  // 所有预设输入都松开/摇杆回中，解除锁定。
  if (!anyPresetInput)
  {
    presetSelectionLatched = false;
    return -1;
  }

  // 当前这次按住已经触发过，不重复触发。
  if (presetSelectionLatched)
  {
    return -1;
  }

  presetSelectionLatched = true;

  // 按用户指定的优先顺序识别16种预设。
  if (ps2x.Button(PSB_L2))       return 0;   // 预设1
  if (ps2x.Button(PSB_L1))       return 1;   // 预设2
  if (ps2x.Button(PSB_PAD_UP))   return 2;   // 预设3
  if (ps2x.Button(PSB_PAD_LEFT)) return 3;   // 预设4
  if (ps2x.Button(PSB_PAD_DOWN)) return 4;   // 预设5
  if (ps2x.Button(PSB_PAD_RIGHT))return 5;   // 预设6
  if (leftStickLeft)             return 6;   // 预设7
  if (leftStickRight)            return 7;   // 预设8
  if (rightStickLeft)            return 8;   // 预设9
  if (rightStickRight)           return 9;   // 预设10
  if (ps2x.Button(PSB_SQUARE))   return 10;  // 预设11
  if (ps2x.Button(PSB_CROSS))    return 11;  // 预设12
  if (ps2x.Button(PSB_CIRCLE))   return 12;  // 预设13
  if (ps2x.Button(PSB_TRIANGLE)) return 13;  // 预设14
  if (ps2x.Button(PSB_R1))       return 14;  // 预设15
  if (ps2x.Button(PSB_R2))       return 15;  // 预设16

  return -1;
}


// ==================================================
// 连接PS2手柄
// ==================================================

void connectPS2()
{
  Serial.println();
  Serial.println("Trying to connect PS2 controller...");

  errorCode = ps2x.config_gamepad(
    PS2_CLK,
    PS2_CMD,
    PS2_CS,
    PS2_DAT,
    false,
    true
  );

  Serial.print("PS2 error code: ");
  Serial.println(errorCode);

  if (errorCode == 0)
  {
    // 每次连接或重新连接成功，都强制回到手动模式。
    stopAllActuators();

    ps2Connected = true;
    ps2ReadFailureCount = 0;
    controlMode = MODE_MANUAL;
    presetMovementActive = false;
    presetSelectionLatched = false;
    activePresetIndex = 0;
    startToggleArmed = false;
    previousStartButton = false;
    stopModeRumble();

    Serial.println("PS2 controller connected successfully.");
    Serial.println("Default mode after connection: MANUAL.");
    Serial.println();
    Serial.println("MANUAL MODE:");
    Serial.println("LEFT PAD DOWN  -> Actuator 1 RETRACT");
    Serial.println("RIGHT CROSS    -> Actuator 1 EXTEND");
    Serial.println("LEFT PAD RIGHT -> Actuator 2 RETRACT");
    Serial.println("RIGHT CIRCLE   -> Actuator 2 EXTEND");
    Serial.println("LEFT PAD UP    -> Actuator 3 RETRACT");
    Serial.println("RIGHT TRIANGLE -> Actuator 3 EXTEND");
    Serial.println("LEFT PAD LEFT  -> Actuator 4 RETRACT");
    Serial.println("RIGHT SQUARE   -> Actuator 4 EXTEND");
    Serial.println();
    Serial.println("START -> Toggle MANUAL / PRESET SELECT mode");
  }
  else
  {
    ps2Connected = false;
    ps2ReadFailureCount = 0;
    controlMode = MODE_MANUAL;
    presetMovementActive = false;
    presetSelectionLatched = false;
    startToggleArmed = false;
    previousStartButton = false;
    stopModeRumble();
    stopAllActuators();

    Serial.println("PS2 connection failed.");
  }
}


// ==================================================
// 初始化
// ==================================================

void setup()
{
  Serial.begin(9600);

  // 初始化四根推杆
  for (uint8_t i = 0; i < 4; i++)
  {
    pinMode(MOTOR_IN1[i], OUTPUT);
    pinMode(MOTOR_IN2[i], OUTPUT);
    pinMode(MOTOR_EN[i], OUTPUT);

    digitalWrite(MOTOR_EN[i], HIGH);
  }

  stopAllActuators();

  Serial.println();
  Serial.println("PS2 + four actuators + VL53L1X starting...");

  // 初始化I2C
  Wire.begin();
  Wire.setClock(100000);

  // 初始化四个测距传感器
  bool allSensorsInitialized = true;

  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
  {
    if (!initializeSensor(i))
    {
      allSensorsInitialized = false;
    }
  }

  if (allSensorsInitialized)
  {
    Serial.println("All four sensors initialized.");
  }
  else
  {
    Serial.println("WARNING: One or more sensors failed.");
  }

  // 等待连续测量启动
  delay(500);

  // 读取一次初始距离
  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
  {
    readSensor(i);
  }

  // 等待PS2接收器和手柄启动
  delay(2500);

  connectPS2();
}


// ==================================================
// 主循环
// ==================================================

void loop()
{
  // 无论手柄是否连接，都继续读取和显示传感器
  updateOneSensor();
  printAllLengths();

  // 手柄未连接时定时重连
  if (!ps2Connected)
  {
    stopAllActuators();
    controlMode = MODE_MANUAL;
    presetMovementActive = false;
    presetSelectionLatched = false;
    startToggleArmed = false;
    previousStartButton = false;
    stopModeRumble();

    if (millis() - lastReconnectAttempt >= RECONNECT_INTERVAL_MS)
    {
      lastReconnectAttempt = millis();
      connectPS2();
    }

    return;
  }

  // 读取手柄状态，并检查手柄是否仍然在线。
  // read_gamepad()返回true表示本次读取到有效的模拟模式数据。
  // Mega根据自己的模式状态决定是否发送震动命令。
  // 第一个参数控制小电机；第二个参数控制大电机强度。
  uint8_t rumbleStrength = getCurrentRumbleStrength();
  bool ps2ReadOK = ps2x.read_gamepad(false, rumbleStrength);

  if (!ps2ReadOK)
  {
    if (ps2ReadFailureCount < 255)
    {
      ps2ReadFailureCount++;
    }

    if (ps2ReadFailureCount >= PS2_MAX_READ_FAILURES)
    {
      // 确认断线后立即停止，并把模式复位为MANUAL。
      stopAllActuators();

      ps2Connected = false;
      controlMode = MODE_MANUAL;
      presetMovementActive = false;
      presetSelectionLatched = false;
      activePresetIndex = 0;
      startToggleArmed = false;
      previousStartButton = false;
      stopModeRumble();

      // 从断线时刻开始计算下一次重连尝试。
      lastReconnectAttempt = millis();

      Serial.println();
      Serial.println("PS2 controller disconnected.");
      Serial.println("All actuators stopped.");
      Serial.println("Mode reset to MANUAL.");
    }

    return;
  }

  // 本次读取正常，清除连续失败计数。
  ps2ReadFailureCount = 0;

  // ==================================================
  // START模式切换：完全由Mega进行按键沿检测
  // ==================================================

  bool startButtonNow = ps2x.Button(PSB_START);

  // 每次连接后，必须先确认START处于松开状态。
  // 在此之前不允许切换模式。
  if (!startToggleArmed)
  {
    previousStartButton = startButtonNow;

    if (!startButtonNow)
    {
      startToggleArmed = true;
      previousStartButton = false;
      Serial.println("START released. Mainboard mode switching enabled.");
    }
  }
  else
  {
    // Mega自己判断从“未按下”变成“按下”的瞬间。
    bool startPressedEdge = startButtonNow && !previousStartButton;
    previousStartButton = startButtonNow;

    if (startPressedEdge)
    {
      if (controlMode == MODE_MANUAL)
      {
        enterPresetSelectionMode();
      }
      else
      {
        enterManualMode();
      }

      return;
    }
  }

  // ==================================================
  // 手动调整四根推杆
  // 保持你当前的按键映射完全不变
  // ==================================================

  if (controlMode == MODE_MANUAL)
  {
    // 推杆1：左侧下键收缩，右侧下键（×）伸长
    bool actuator1Extend  = ps2x.Button(PSB_CROSS);
    bool actuator1Retract = ps2x.Button(PSB_PAD_DOWN);

    // 推杆2：左侧右键收缩，右侧右键（○）伸长
    bool actuator2Extend  = ps2x.Button(PSB_CIRCLE);
    bool actuator2Retract = ps2x.Button(PSB_PAD_RIGHT);

    // 推杆3：左侧上键收缩，右侧上键（△）伸长
    bool actuator3Extend  = ps2x.Button(PSB_TRIANGLE);
    bool actuator3Retract = ps2x.Button(PSB_PAD_UP);

    // 推杆4：左侧左键收缩，右侧左键（□）伸长
    bool actuator4Extend  = ps2x.Button(PSB_SQUARE);
    bool actuator4Retract = ps2x.Button(PSB_PAD_LEFT);

    controlActuator(
      0,
      actuator1Extend,
      actuator1Retract
    );

    controlActuator(
      1,
      actuator2Extend,
      actuator2Retract
    );

    controlActuator(
      2,
      actuator3Extend,
      actuator3Retract
    );

    controlActuator(
      3,
      actuator4Extend,
      actuator4Retract
    );
  }

  // ==================================================
  // 选择16种预设位置
  // ==================================================

  else
  {
    int8_t selectedPreset = detectPresetSelection();

    if (selectedPreset >= 0)
    {
      Serial.print("Preset key detected: ");
      Serial.println(selectedPreset + 1);
      startPresetMovement((uint8_t)selectedPreset);
    }

    updatePresetMovement();
  }

  delay(20);
}