/*
 * HC-SR04 diagnostic over the verified P4 <-> Mega USB bridge.
 *
 * Wiring (Arduino Mega 2560):
 *   HC-SR04 TRIG -> D51
 *   HC-SR04 ECHO -> D50
 *   HC-SR04 VCC  -> 5V
 *   HC-SR04 GND  -> GND
 *
 * Safety scope: distance measurement + USB serial reporting only.
 * No motor, actuator, PWM, GPIO driver, or landing-control output.
 *
 * Commands, each terminated by LF:
 *   GET_DISTANCE     take one reading and print one SR04 record
 *   STREAM <Hz>      stream 1..10 readings per second
 *   STREAM OFF       stop streaming
 *   PING             reply PONG
 */

#include <string.h>
#include <stdlib.h>

constexpr uint8_t TRIG_PIN = 51;
constexpr uint8_t ECHO_PIN = 50;
constexpr uint32_t ECHO_RISE_TIMEOUT_US = 30000UL;
constexpr uint32_t ECHO_HIGH_TIMEOUT_US = 30000UL;
constexpr int32_t MIN_DISTANCE_MM = 20;
constexpr int32_t MAX_DISTANCE_MM = 4000;

struct Sr04Reading {
  int32_t distance_mm;
  uint32_t pulse_us;
  const char* status;
};

uint32_t sequence_number = 0;
uint32_t stream_interval_ms = 0;
uint32_t last_stream_ms = 0;
char command_line[48];
size_t command_length = 0;

Sr04Reading readSr04() {
  // A high Echo before triggering is an electrical/wiring fault, not a range.
  if (digitalRead(ECHO_PIN) == HIGH) {
    return { -1, 0, "echo_stuck_high" };
  }

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // pulseIn waits for rising then falling edge. A zero result is no complete
  // echo within 30 ms; inspect the pin once more to distinguish stuck high.
  const uint32_t pulse_us = pulseIn(ECHO_PIN, HIGH, ECHO_RISE_TIMEOUT_US + ECHO_HIGH_TIMEOUT_US);
  if (pulse_us == 0) {
    return { -1, 0, digitalRead(ECHO_PIN) == HIGH ? "echo_stuck_high" : "no_echo_rise" };
  }
  if (pulse_us > ECHO_HIGH_TIMEOUT_US) {
    return { -1, pulse_us, "echo_too_long" };
  }

  // Sound speed approximately 340 m/s: distance_mm = pulse_us * 0.17.
  const int32_t distance_mm = (int32_t)((pulse_us * 17UL + 50UL) / 100UL);
  if (distance_mm < MIN_DISTANCE_MM || distance_mm > MAX_DISTANCE_MM) {
    return { -1, pulse_us, "range_invalid" };
  }
  return { distance_mm, pulse_us, "ok" };
}

void reportReading() {
  const Sr04Reading reading = readSr04();
  ++sequence_number;
  Serial.print("SR04 seq=");
  Serial.print(sequence_number);
  Serial.print(" t_ms=");
  Serial.print(millis());
  Serial.print(" distance_mm=");
  Serial.print(reading.distance_mm);
  Serial.print(" pulse_us=");
  Serial.print(reading.pulse_us);
  Serial.print(" status=");
  Serial.println(reading.status);
}

void processCommand(char* command) {
  if (strcmp(command, "PING") == 0) {
    Serial.println("PONG");
    return;
  }
  if (strcmp(command, "GET_DISTANCE") == 0) {
    reportReading();
    return;
  }
  if (strcmp(command, "STREAM OFF") == 0) {
    stream_interval_ms = 0;
    Serial.println("SR04 STREAM=OFF");
    return;
  }
  if (strncmp(command, "STREAM ", 7) == 0) {
    const long hz = strtol(command + 7, nullptr, 10);
    if (hz < 1 || hz > 10) {
      Serial.println("ERR:STREAM_HZ_MUST_BE_1_TO_10");
      return;
    }
    stream_interval_ms = 1000UL / (uint32_t)hz;
    last_stream_ms = 0;
    Serial.print("SR04 STREAM_HZ=");
    Serial.println(hz);
    return;
  }
  Serial.print("ERR:UNKNOWN_COMMAND:");
  Serial.println(command);
}

void pollCommands() {
  while (Serial.available() > 0) {
    const char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      command_line[command_length] = '\0';
      if (command_length > 0) processCommand(command_line);
      command_length = 0;
    } else if (command_length < sizeof(command_line) - 1) {
      command_line[command_length++] = ch;
    } else {
      command_length = 0;
      Serial.println("ERR:LINE_TOO_LONG");
    }
  }
}

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  Serial.begin(115200);
  Serial.println("MEGA_SR04_READY trig=D51 echo=D50 range_mm=20..4000");
}

void loop() {
  pollCommands();
  if (stream_interval_ms > 0 && (uint32_t)(millis() - last_stream_ms) >= stream_interval_ms) {
    last_stream_ms = millis();
    reportReading();
  }
}
