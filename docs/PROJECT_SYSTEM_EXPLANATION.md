# Mars Lander Project — System Architecture and Engineering Explanation

## 1. Mission Context

The project develops a four-legged landing platform for a Mars-exploration scenario in which a lander must settle on locally uneven terrain. A rigid landing platform can transfer terrain height differences directly into body roll and pitch. Large tilt can compromise camera pointing, payload stability, solar-array orientation, and the useful range of the landing legs. The engineering objective is therefore to estimate or measure the relative elevation of four support regions and independently regulate the four landing legs so that the platform reaches a controlled attitude.

The current prototype separates this long-term objective into a safe operational layer and a research layer. The operational layer provides deterministic leg control through a source-faithful physical Remote and a validated inertial mode. The research layer develops camera calibration, depth reconstruction, ultrasonic depth scaling, and foot-region terrain analysis. This separation prevents an unvalidated AI or vision estimate from commanding hardware.

## 2. Architecture Overview

```text
                       Windows PC
       ┌─────────────────────────────────────┐
       │ Console: mode, Remote targets,       │
       │ status, video/IMU display, CV study  │
       └──────────────────┬──────────────────┘
                          │ Wi-Fi / HTTP / TCP
                          ▼
                     ESP32-P4 Gateway
       ┌─────────────────────────────────────┐
       │ OV5647 camera / BNO085 / Wi-Fi      │
       │ USB Host / CH340 / Mega protocol    │
       │ HTTP /board, /health, /cmd          │
       └──────────────────┬──────────────────┘
                          │ USB Host to CH340
                          ▼
                    Arduino Mega 2560
       ┌─────────────────────────────────────┐
       │ PS2 Remote / four VL53L1X sensors   │
       │ TCA9548A / motor outputs / SR04     │
       │ local feedback, timeout, STOP/ESTOP │
       └───────────┬───────────┬─────────────┘
                   ▼           ▼
          Four linear actuators   Four corner feet
```

The Mega is the safety-critical actuator authority. The PC and P4 can request a command, but they cannot replace the Mega's local sensor feedback, range checks, staleness checks, heartbeat timeout, or output-disable behavior. This allocation is deliberate: wireless transport, computer vision, or the Windows user interface can fail without removing local actuator protection.

## 3. Mechanical and Sensor Architecture

The lander platform is approximately \(200\times200\) mm, with an independently driven actuator at each corner. Each actuator has a VL53L1X time-of-flight sensor measuring the relevant local position coordinate. The four equal-address sensors are selected through a TCA9548A I\textsuperscript{2}C multiplexer.

The canonical physical mapping is:

```text
A1 = RL, rear-left
A2 = RR, rear-right
A3 = FR, front-right
A4 = FL, front-left

UI / solver order: FL, FR, RL, RR
```

The actuator direction convention is defined in the actual feedback coordinate:

```text
EXTEND  increases the associated VL53 reading.
RETRACT decreases the associated VL53 reading.
```

The camera is centered and points downward. Its optical center is approximately 50 mm below the upper-joint plane. The BNO085 provides orientation information. The HC-SR04 is physically connected to the Mega and is used only as a possible absolute scale anchor for visual depth research, not as a direct command variable for leg height.

## 4. Stage-1 Control Design

### 4.1 Why Stage 1 does not use automatic CV actuation

A visual depth map may be temporally stable while still being geometrically wrong for a particular lens, lighting condition, material, or terrain texture. It can also fail when one foot region is occluded or outside the visible field. Therefore, the project explicitly prevents automatic camera/depth output from entering the actuator command channel during Stage 1.

The Console label ``CV Control'' is retained for the user interface, but its Stage-1 implementation is a Remote owner. This mode enables a physical PS2 Remote, the original preset-selection behavior, and controlled direct entry of four absolute VL53 targets. It does not mean that camera output can move the legs.

### 4.2 Faithful Remote behavior

The source Remote program was hash-locked and used as the behavior baseline. It preserves the PS2 pin assignment, manual mappings, START-button transition between manual and preset selection, 16 preset vectors, preset order, connection retry behavior, and completion logic. The Remote is not merely simulated by the Console.

Manual commands use held buttons: releasing a button stops the corresponding leg. Preset mode uses 16 pre-defined four-leg target vectors. Target completion requires two independent in-tolerance sensor confirmations; the action times out after 30 s. Three consecutive Remote read failures declare a disconnection and stop the outputs.

The Console provides an additional Remote input surface. It accepts FL/FR/RL/RR targets in the actual VL53 coordinate, maps them to A4/A3/A1/A2, and sends a `SET_REMOTE` request through the P4. Console target motion temporarily pauses PS2 command processing. On completion or cancellation, the Remote session returns to PS2 operation.

### 4.3 IMU mode

IMU mode is a distinct owner. It uses BNO085 attitude feedback and a pre-validated coordinate transformation to reduce measured roll and pitch. It cannot be active together with Remote mode. A mode change stops the current owner before the next owner may be armed.

The IMU loop converts attitude error into coordinated differential leg adjustments. It seeks to change the support-plane orientation rather than apply an arbitrary common vertical motion. Existing PID and rate-limit behavior remain in the IMU pathway; Stage-1 Remote work must not replace or retune this controller without separate commissioning evidence.

## 5. Local Safety Model

The Mega implements the following state model:

```text
LOCKED
  all motor outputs low; movement blocked.

REMOTE_MANUAL / REMOTE_PRESET
  physical Remote enabled after explicit ARM REMOTE.

ARMED / ACTIVE
  IMU owner enabled after explicit ARM IMU.

FAULT / ESTOP
  outputs low; normal motion blocked until correct recovery action.
```

Key local protections are:

- four valid, fresh VL53 position readings;
- active coordinate upper bound of 400 mm and current lower bound of 0 mm;
- 1200 ms sensor freshness timeout;
- 2 s heartbeat timeout for host-owned sessions;
- explicit STOP and latched ESTOP;
- all outputs low in LOCKED, FAULT, and ESTOP;
- 120 ms break-before-make for direction reversal;
- local target validation before motor output;
- full-group rejection of invalid targets.

The lower bound was changed from 50 mm to 0 mm after a physical channel reported a valid reading below 50 mm. This was a targeted correction to an over-restrictive range rule, not removal of the sensor-validity, stale-data, upper-travel, heartbeat, or emergency-stop protections.

## 6. P4 Gateway and PC Console

The ESP32-P4 is the communication gateway. It captures camera images, receives BNO085 data, serves the network interfaces, manages the USB Host connection to the Mega's CH340 interface, and serializes all commands sent to the Mega.

The P4 provides a strict identity endpoint:

```text
GET /board  →  board=ESP32-P4 plus current DHCP address
```

The Windows Console must verify this identity rather than assume a historical IP address. `/health` reports the current Mega state, four-leg feedback, Remote state, PS2 connection state, and serial receive counters. These counters are important because a stale status object is not proof that the Mega application is currently running.

The P4 has two corresponding Remote command requests:

```text
remote_arm
remote_set_absolute(seq, FL, FR, RL, RR)
```

The P4 sends these through the same serialized CH340 command path used by the IMU commands. There is no parallel write path that could corrupt command lines or sequence ordering.

## 7. Vision and Depth Research Path

The project has completed fisheye intrinsic calibration using 36 views. The accepted calibration has RMS reprojection error of approximately 0.717 pixels. An external camera-to-board pose collection also produced a mean reprojection error of approximately 0.345 pixels and a camera-to-plane distance of approximately 354.73 mm. However, the front/back orientation ambiguity is not yet resolved, so the current external pose is not a control input.

The intended depth pipeline is:

```text
camera image
→ fisheye-aware geometry
→ model relative depth
→ stable local terrain estimate
→ SR04 scale anchor when its independent-sample window is stable
→ four foot-region elevation estimates
→ zero-common-mode relative leg correction proposal
```

The relative leg correction must satisfy

\[
\delta_i=L_i-\frac{1}{4}\sum_{j=1}^{4}L_j,
\qquad \sum_i\delta_i=0.
\]

This rejects the physically unhelpful case in which all legs are moved by the same amount merely because the entire estimated terrain map has a common scale or offset error. Each correction is bounded to \(\pm100\) mm relative to the mean. If a boundary is reached, the whole differential vector must be scaled by one common factor; independently clipping individual legs would distort the desired platform slope.

Before Stage-2 actuation can be considered, the project still needs flat-plane tests, measured 10/20/30/50 mm step tests, front/back pose disambiguation, repeated scale-anchor validation, and staged low-amplitude hardware tests.

## 8. Deployment and Verification Strategy

Firmware is built with the real Windows toolchains and flashed through the P4 debug interface on COM5. Mega application updates are performed through a controlled temporary P4 bridge. The update procedure verifies the ATmega2560 signature, writes only the application area, and reads every written flash page back for comparison. The P4 production image is restored after the bridge operation.

The safe verification sequence is always:

```text
1. Verify P4 identity.
2. Verify increasing Mega RX counters.
3. Verify LOCKED, fault=none, all four VL53 valid.
4. Verify target=0 and applied=0.
5. Arm only one owner explicitly.
6. Begin with no-motion and very small displacement tests.
7. STOP and re-check LOCKED after each test.
```

This architecture allows the project to make measurable progress on terrain sensing and control while maintaining a clear boundary between research output and real actuator authority.
