# Mars Lander Project — Engineering Handover

**Updated:** 2026-08-07 (CST)  
**Project root:** `E:\Chenyuewei\University\Freshman\summer\ENGR1000\p2\project`  
**Purpose:** preserve the hardware mapping, currently deployed firmware behavior, safety architecture, validation evidence, and next-stage commissioning sequence for the four-legged Mars landing platform.

---

## 1. System Purpose and Current Development Boundary

The project is a four-legged, terrain-adaptive Mars lander prototype. A downward-facing camera and an IMU support terrain research and attitude estimation; four independently controlled linear actuators provide landing-platform adjustment. The intended long-term behavior is to maintain a stable platform over unequal foot elevations.

The project is deliberately divided into two control stages:

```text
Stage 1 — deployed and controllable
  “CV Control” console mode = Remote owner, not automatic CV actuation.
  - faithful PS2 manual control and 16 source-program presets;
  - direct Console FL/FR/RL/RR absolute VL53 target entry;
  - Mega local four-leg position feedback and safety enforcement.

  “IMU Control” console mode = validated IMU attitude-control owner.
  - mutually exclusive with Remote;
  - retains the existing BNO085 attitude-control pathway.

Stage 2 — research only until separately validated
  Camera calibration → depth/terrain reconstruction → SR04 scale anchoring
  → four-foot relative elevation estimation → low-amplitude CV closed loop.

  `CV_CONTROL_VALIDATED = False` remains mandatory.
```

A computer-vision result, depth model, video stream, or SR04 reading must **not** enter a leg-command path in Stage 1.

---

## 2. Current Deployed Hardware State

### 2.1 Latest verified no-motion state

After the latest deployment and P4 recovery, repeated `/health` observations showed:

```text
P4 identity:  ESP32-P4 at 192.168.137.106
Mega state:   LOCKED
fault:        none
armed:        false
active:       false
target:       0,0,0,0
applied:      0,0,0,0
Mega RX line counter: increasing
```

Observed position feedback during the final no-motion verification was valid on all four channels:

```text
A1 = approximately 159–163 mm
A2 = approximately 94–96 mm
A3 = approximately 219–222 mm
A4 = approximately 143–145 mm
```

These values are installation-dependent measurements, not calibration constants.

### 2.2 Boards and links

| Layer | Hardware | Responsibility |
|---|---|---|
| High-level gateway | ESP32-P4 Function EV Board v1.3 | camera, BNO085, Wi-Fi/HTTP/TCP, USB Host, Mega command forwarding |
| Actuator controller | Arduino Mega 2560 + CH340 | four VL53 feedback loops, PS2 Remote behavior, motor outputs, STOP/ESTOP/watchdog |
| Sensor multiplexer | TCA9548A | isolates the four equal-address VL53L1X sensors |
| Inertial sensor | BNO085 | platform attitude for IMU mode |
| Camera | OV5647 | 800×800 downward-facing fisheye image stream |
| Range anchor | HC-SR04 on Mega | depth-scale research input only; never a Stage-1 leg target |
| PC application | Windows Console | mode selection, status display, Remote absolute-target entry, visualization and research tools |

### 2.3 USB identities

```text
P4 debug/programming interface:
  USB-Enhanced-SERIAL CH343, COM5
  VID:PID = 1A86:55D3

Mega downstream USB interface:
  CH340, VID:PID = 1A86:7523

Mega target signature:
  ATmega2560 = 1E9801
```

The P4 USB-A/OTG host connects to the Mega USB-B/CH340 device. P4-to-Mega commands must remain serialized through the single P4 transmit mutex.

---

## 3. Physical Four-Leg Mapping

The following mapping was established by bounded one-leg identification. It is authoritative.

```text
Mega A1 / TCA channel 0 / D22,D23,EN30 = RL (rear-left)
Mega A2 / TCA channel 1 / D24,D25,EN31 = RR (rear-right)
Mega A3 / TCA channel 2 / D26,D27,EN32 = FR (front-right)
Mega A4 / TCA channel 3 / D28,D29,EN33 = FL (front-left)

UI/solver order: FL, FR, RL, RR
Mega order:      A1, A2, A3, A4
Conversion:      FL→A4, FR→A3, RL→A1, RR→A2
```

Direction convention:

```text
EXTEND  → associated VL53 reading increases
RETRACT → associated VL53 reading decreases
```

Never pass a UI/solver array directly to Mega array positions without this remapping.

---

## 4. Mega Hybrid Firmware: Current Behavior

### 4.1 Source-faithful Remote controller

The hybrid firmware preserves the user-supplied Remote source as the behavioral reference. Immutable original reference:

```text
reference/111_1_original.ino
SHA256 = 1973c42c856dd61e18a53a6e8ca50d0e7503ce2a498bed8b9e75497d92f651f8
```

The deployed firmware preserves:

- PS2 wiring: DAT=D49, CMD=D47, CLK=D46, CS=D48;
- START toggle between `MANUAL` and `PRESET` selection modes;
- the original eight manual directional button mappings;
- the original 16 preset target vectors, values, and selection priority;
- three consecutive PS2 read failures before disconnect declaration;
- PS2 reconnect attempts every 2 s;
- preset tolerance of ±3 mm, two consecutive in-tolerance confirmations, and 30 s timeout.

The project-level automated contract compares the original and hybrid source for the presets, mappings, timing constants, reconnect behavior, and ownership boundary.

### 4.2 Ownership states

```text
LOCKED
  ALL_LOW; no PS2, Console, or IMU movement is permitted.

REMOTE_MANUAL / REMOTE_PRESET
  PS2 Remote is enabled after explicit `ARM REMOTE`.
  A Console absolute target temporarily suspends PS2 commands while it runs.

ARMED / ACTIVE
  IMU owner is enabled after explicit `ARM IMU`.
  PS2 Remote is suspended.

FAULT / ESTOP
  ALL_LOW; fault or emergency condition blocks normal movement.
```

Mode switching must stop the current owner before a new owner is armed. `STOP` and `ESTOP` bypass normal command authorization.

### 4.3 Active local safety functions

The following protections remain enabled and must not be removed during normal work:

```text
- four VL53 readings must be valid and fresh;
- maximum authoritative VL53 coordinate = 400 mm;
- current deployed lower coordinate = 0 mm;
- 2 s heartbeat timeout for host-owned Remote and IMU sessions;
- stale sensor timeout = 1200 ms;
- explicit STOP; latched ESTOP;
- hard all-output-low in LOCKED, FAULT, and ESTOP states;
- 120 ms break-before-make only when changing motor direction;
- non-blocking concurrent four-leg position regulation;
- full-group invalid target rejection, rather than silently altering an invalid command.
```

The lower coordinate was changed from 50 mm to 0 mm because a real valid sensor reading below 50 mm had caused `sensor_invalid_or_range`. This change does **not** remove sensor-validity, staleness, upper-limit, heartbeat, STOP, or ESTOP protection.

### 4.4 Remote protocol

```text
ARM REMOTE
HEARTBEAT <seq>
SET_REMOTE <seq> <FL> <FR> <RL> <RR>
STOP
ESTOP
STATUS
GET_DISTANCE
```

`SET_REMOTE` is in UI order `FL,FR,RL,RR` and uses direct absolute VL53 coordinates in the currently deployed range `[0,400]` mm. The Mega remaps this to physical A4,A3,A1,A2.

IMU commands remain separate:

```text
ARM IMU
HEARTBEAT <seq>
SET_DELTA <seq> <FL> <FR> <RL> <RR>
STOP
ESTOP
```

`SET_DELTA` is ARM-relative and still uses the bounded delta protocol. The Mega evaluates the resulting absolute target locally.

---

## 5. P4 Firmware and Network Interface

### 5.1 P4 production behavior

The ESP32-P4 production application provides:

```text
GET /board
  Board identity and current DHCP address.
  Required identity check: board == "ESP32-P4" and returned IP matches probe host.

GET /health
  P4 network, Mega, four-leg, SR04, Remote, and diagnostic status.

POST /cmd {"remote_arm": true}
  Forwards ARM REMOTE.

POST /cmd {"remote_set_absolute": true,
           "seq": N, "fl": ..., "fr": ..., "rl": ..., "rr": ...}
  Forwards SET_REMOTE with four absolute VL53 targets in [0,400] mm.

Existing IMU commands
  imu_arm, imu_set_delta, imu_heartbeat, imu_stop, imu_estop.
```

`/health.mega_imu` includes remote telemetry:

```text
remote_active
remote_source = ps2 | console | none
remote_preset = 0..15 or -1
remote_target_fl/fr/rl/rr
ps2_connected
remote_submode = manual | preset
```

### 5.2 Current network endpoints

| Endpoint | Function |
|---|---|
| TCP 5000 | JPEG frames: `[u32 little-endian length][JPEG bytes]` |
| TCP 5001 | JSONL latest BNO085 sample stream |
| HTTP `/board` | strict board identity and DHCP discovery |
| HTTP `/health` | P4/Mega/Remote/leg diagnostics |
| HTTP `/imu` | current IMU diagnostic state |

The current P4 address was last verified as `192.168.137.106`; do not hard-code it permanently. Use `/board` discovery.

### 5.3 CH340 session recovery

The production P4 firmware performs the proven sequence after CH340 opening:

```text
set_control_line_state(false, true)
wait 100 ms
set_control_line_state(true, true)
wait 6500 ms
begin Mega STATUS / GET_DISTANCE polling
```

It also reports `rx_byte_count` and `rx_line_count`. If the byte counter does not advance, the P4 is not receiving Mega application output; do not diagnose Console logic before resolving the transport/application layer.

---

## 6. Windows Console

Run the Console only with:

```text
pc_viewer\.venv_win\Scripts\python.exe
pc_viewer\.venv_win\Scripts\pythonw.exe
```

The UI has three modes:

```text
CV Control (Stage 1 Remote)
  - ARM button: ARM Remote
  - FL/FR/RL/RR absolute target entries: editable
  - Apply Remote target button: enabled
  - PS2 manual/preset remains the primary Remote interaction surface
  - CV, depth, video, and SR04 are preview/research-only inputs.

IMU Control
  - ARM button: ARM IMU
  - Remote target entries: read-only
  - Remote Apply button: disabled
  - P4 TCP 5001 attitude stream is required by the IMU loop itself.

Locked
  - Remote entries: read-only
  - Remote Apply: disabled
  - ARM: disabled.
```

Remote command availability does not depend on SR04, camera TCP 5000, depth model load, CV capture count, CV candidate age, or `CV_CONTROL_VALIDATED`. It does depend on P4/Mega communication, valid four-leg feedback, owner state, and hard safety status.

Before launching a new Console instance, close stale `lander_console.py` processes. Two stale processes previously re-armed a Remote session after P4 recovery; this was resolved by issuing `STOP`, confirming `LOCKED`, and closing both processes.

---

## 7. Camera, IMU, SR04, and Vision Research

### 7.1 Camera calibration

The replacement camera is an 800×800 fisheye camera. The accepted calibration result is:

```text
Calibration set: 36 views
Board: 11×8 squares, 10×7 inner corners, 15 mm square size
RMS reprojection error: 0.717 px
Mean reprojection error: 0.562 px
Maximum reprojection error: 1.947 px
Accepted for geometry: true

K ≈ [[638.075, 0, 613.459],
     [0, 636.365, 452.471],
     [0,   0,       1   ]]
D ≈ [-0.06099, 0.09688, -0.14361, 0.06626]
```

Results are stored under:

```text
wifi_test/calibration_runs/2026-08-04-fisheye-10x7-15mm/
```

External-pose collection reached 20/20 usable samples:

```text
Mean reprojection error: 0.345 px
Maximum reprojection error: 0.366 px
Camera-to-board-plane distance: 354.73 mm
Optical-axis inclination: 3.66 degrees
```

The platform front/back 180° ambiguity remains unresolved. Do not use this pose for control orientation until that ambiguity is eliminated.

### 7.2 SR04 role

The HC-SR04 is wired to Mega:

```text
TRIG = D53
ECHO = D52 / PB1 / PCINT1
```

It is a scale anchor for depth research only. A stable window requires recent independent samples, a median, minimum count, and spread/MAD checks. Its accepted use is:

\[
\mathrm{scale}=
\frac{\mathrm{stable\ SR04\ median}}
{\mathrm{model\ depth\ median\ in\ principal-point\ ROI}},
\qquad
D_{\mathrm{corrected}}=\mathrm{scale}\cdot D_{\mathrm{model}}.
\]

If the SR04 stability test fails, use `metric_source=model_only`; do not reuse an old scale anchor. SR04 distance must never be converted directly into a common leg extension.

### 7.3 CV terrain semantics

Future CV control may use only relative terrain differences across the four predicted foot regions:

\[
\delta_i=L_i-\frac{1}{4}\sum_{j=1}^{4}L_j,
\qquad
\sum_i\delta_i=0.
\]

The current research constraints are:

```text
- each relative correction limited to ±100 mm;
- boundary handling uses one common scale factor, never independent clipping;
- a common terrain-height offset must not change the command;
- NaN/unobservable foot regions reject the plan;
- vision output remains preview-only until physical plane/step validation is complete.
```

A real scale-correction script processed 16 valid anchored frames with an observed scale approximately 0.526–0.607, but the intended 30-frame acquisition was not completed because TCP 5000 permits one client. Do not claim complete 30-frame acceptance yet.

---

## 8. Firmware Artifacts and Build Evidence

### Mega hybrid source

```text
usb_cdc_mega_test/faithful_remote_hybrid/
  faithful_remote_hybrid/faithful_remote_hybrid.ino
  reference/111_1_original.ino
  test_faithful_remote_contract.py
```

Latest range-0-to-400 Mega application image:

```text
SHA256 = da51089c186d3912c7e228fabce591849e693fdf4ee7221c5b981866abb54076
ATmega2560 application payload = 22,380 bytes
Pages written = 88
Verification = every page read back after write
```

### P4 production source

```text
wifi_test/main/main.c
wifi_test/main/mega_sr04.c
wifi_test/main/mega_sr04.h
```

Current build passed ESP-IDF v5.4 and generated `wifi_test.bin` with approximately 21% free application partition space. P4 flashing verified the bootloader, partition table, and application image with `Hash of data verified` for each region.

### Tests last run successfully

```text
PC Console + terrain/control suite: 124 tests
Faithful Remote source contract: 9 tests
P4 protocol contract: 12 tests
Legacy Mega controller contract: 10 tests
```

---

## 9. Safe Commissioning Sequence

Use the following sequence after any firmware, cable, or mechanical change:

1. Confirm no stale Console process is running.
2. Read `/board`; verify `board=ESP32-P4` and the current DHCP address.
3. Read `/health` repeatedly. Require increasing Mega RX counters, `LOCKED`, `fault=none`, valid A1--A4, zero target, and zero applied values.
4. If any fault occurs, send `STOP`; do not arm around a fault.
5. For a first Remote test, arm Remote with no button pressed and no Console target.
6. Confirm `REMOTE_MANUAL` or `REMOTE_PRESET`, `remote_active=false`, and no motor output.
7. Perform a same-current-value Console target or a verified no-motion preset selection before requesting motion.
8. Test one small 2--3 mm change at a time; then STOP and verify `LOCKED`.
9. Commission PS2 manual and preset behavior separately from IMU behavior.
10. Commission IMU only after Remote STOP, explicit IMU ARM, and live TCP 5001 attitude verification.

Never start a movement experiment by testing all four legs, a large preset, or an unverified CV output.

---

## 10. Open Work

1. Diagnose the full PS2 electrical connection if `ps2_connected=false`; a disconnected Remote must remain harmless.
2. Complete Stage-1 physical verification of selected source presets and direct Console absolute targets using small, supervised movements.
3. Re-confirm the IMU closed-loop behavior after the Hybrid Mega firmware change; preserve the previously accepted installation transform and PID semantics.
4. Resolve the external-pose front/back ambiguity.
5. Complete physical flat-plane and 10/20/30/50 mm step validation for the depth pipeline.
6. Complete a dedicated 30-frame SR04-anchored depth acquisition after releasing TCP 5000 from other clients.
7. Do not enable automatic CV leg actuation until all calibration, terrain, relative-delta, and staged hardware acceptance gates are documented as passed.
