#include <Wire.h>

// Read-only Mega 2560 I2C diagnostic. No motor control functions exist.
// Canonical bus: SDA=D20, SCL=D21.
const uint8_t MOTOR_IN1[4] = {22, 24, 26, 28};
const uint8_t MOTOR_IN2[4] = {23, 25, 27, 29};
const uint8_t MOTOR_EN[4]  = {30, 31, 32, 33};

void forceAllMotorPinsLow() {
  for (uint8_t i = 0; i < 4; ++i) {
    pinMode(MOTOR_IN1[i], OUTPUT); digitalWrite(MOTOR_IN1[i], LOW);
    pinMode(MOTOR_IN2[i], OUTPUT); digitalWrite(MOTOR_IN2[i], LOW);
    pinMode(MOTOR_EN[i], OUTPUT);  digitalWrite(MOTOR_EN[i], LOW);
  }
}

void scanBus() {
  forceAllMotorPinsLow();
  Serial.print("I2C_SCAN_BEGIN SDA_D20="); Serial.print(digitalRead(20));
  Serial.print(" SCL_D21="); Serial.println(digitalRead(21));
  uint8_t found = 0;
  for (uint8_t address = 0x08; address <= 0x77; ++address) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C_ACK address=0x");
      if (address < 16) Serial.print('0');
      Serial.println(address, HEX);
      ++found;
    } else if (error == 4) {
      Serial.print("I2C_UNKNOWN_ERROR address=0x");
      if (address < 16) Serial.print('0');
      Serial.println(address, HEX);
    }
  }
  Serial.print("I2C_SCAN_END found="); Serial.print(found);
  Serial.print(" SDA_D20="); Serial.print(digitalRead(20));
  Serial.print(" SCL_D21="); Serial.println(digitalRead(21));
}

void setup() {
  Serial.begin(115200);
  forceAllMotorPinsLow();
  delay(1200);
  Serial.println("MEGA_I2C_SCAN_DIAGNOSTIC MOTION_LOCKED ALL_OUTPUTS_LOW");
  Wire.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(25000, true);
  scanBus();
}

void loop() {
  forceAllMotorPinsLow();
  if (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == 'S' || ch == 's') scanBus();
  }
  delay(20);
}
