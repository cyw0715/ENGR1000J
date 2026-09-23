/*
 * Arduino Mega USB CDC echo for ESP32-P4-NANO Stage-1 test.
 * No GPIO, PWM, motor, or actuator operation.
 */

void setup() {
  Serial.begin(115200);
  Serial.println("MEGA_USB_READY");
}

void loop() {
  static char line[64];
  static size_t length = 0;

  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      line[length] = '\0';
      if (strcmp(line, "PING") == 0) {
        Serial.println("PONG");
      } else {
        Serial.print("ECHO:");
        Serial.println(line);
      }
      length = 0;
    } else if (length < sizeof(line) - 1) {
      line[length++] = ch;
    } else {
      length = 0;
      Serial.println("ERR:LINE_TOO_LONG");
    }
  }
}
