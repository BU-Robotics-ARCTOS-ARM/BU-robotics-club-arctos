# Arctos Robotic Arm — Closed Loop Specifications

Reference document for development. Used LLM to compile from the Arctos docs and MKS SERVO42D/57D_CAN V1.0.5 user manual.

---

## 1. General Specifications

| Parameter         | Value                        |
| ----------------- | ---------------------------- |
| Degrees of Freedom | 6                           |
| Maximum Reach     | 600 mm                      |
| Payload           | 1 kg                        |
| Control System    | Closed-loop (encoder feedback) |
| Communication Bus | CAN (standard frames)       |
| CAN Bitrate       | 500 Kbps (default)          |
| CAN Interface     | CANable V2 (USB-to-CAN)     |
| Power Supply      | 12–24 V DC                  |
| Encoder Resolution | 16384 counts/revolution (14-bit, 0x0000–0x3FFF) |

---

## 2. Joint / Axis Mapping

| Axis | Name       | Joint   | Link       | CAN ID | Driver     | Motor   |
| ---- | ---------- | ------- | ---------- | ------ | ---------- | ------- |
| X    | Base       | joint1  | Link_1_1   | 0x01   | MKS 57D    | NEMA 23 |
| Y    | Shoulder   | joint2  | Link_2_1   | 0x02   | MKS 57D    | NEMA 23 |
| Z    | Elbow      | joint3  | Link_3_1   | 0x03   | MKS 42D    | NEMA 17 |
| A    | Wrist 1    | joint4  | Link_4_1   | 0x04   | MKS 42D    | NEMA 17 |
| B    | Wrist 2    | joint5  | Link_5_1   | 0x05   | MKS 42D    | NEMA 17 |
| C    | Wrist 3    | joint6  | Link_6_1   | 0x06   | MKS 42D    | NEMA 17 |

> **Note:** The Arctos specs page lists B-axis as CAN ID 07, but the wiring diagram (more recent) shows 05. This table uses 05 per the wiring diagram — verify on your hardware's OLED display to confirm.

---

## 3. Gear Ratios

| Axis | Gear Ratio | Gearbox Type       | Notes                          |
| ---- | ---------- | ------------------ | ------------------------------ |
| X    | 1:13.5     | Belt reduction     | GT2 belt and pulley            |
| Y    | 1:150      | Cycloidal (2-stage)| 1:24 per stage × ~6.25         |
| Z    | 1:150      | Cycloidal (2-stage)| Same as Y                      |
| A    | 1:48       | Compound Planetary | Lighter, noisier               |
| B    | 1:67.82    | Compound Planetary | Same type as C                 |
| C    | 1:67.82    | Compound Planetary | Same type as B                 |

### Angle-to-Encoder Conversion

One full motor revolution = 16384 encoder counts (0x4000).

To rotate a joint by `θ` degrees, the motor must move:

```
encoder_counts = (θ / 360) × gear_ratio × 16384
```

| Axis | Encoder counts per joint degree |
| ---- | ------------------------------- |
| X    | (13.5 × 16384) / 360 ≈ 614.4   |
| Y    | (150 × 16384) / 360 ≈ 6,826.7  |
| Z    | (150 × 16384) / 360 ≈ 6,826.7  |
| A    | (48 × 16384) / 360 ≈ 2,184.5   |
| B    | (67.82 × 16384) / 360 ≈ 3,086.5|
| C    | (67.82 × 16384) / 360 ≈ 3,086.5|

---

## 4. Denavit-Hartenberg Parameters

The arm follows a standard 6R (6 revolute joints) configuration with a spherical wrist (joints 4, 5, 6 intersect at a common point).

> **Convention:** These parameters use the **standard (classic) DH convention**, not the modified (Craig) convention. This matters when building transformation matrices — the order of rotations and translations differs between conventions. If using a library like `ikpy` or `robotics-toolbox-python`, make sure you select the matching convention.

| Joint (i) | q_i | α_i (°) | a_i (mm) | θ_i (°) | d_i (mm) | sin α_i | cos α_i |
| ---------- | --- | ------- | -------- | ------- | -------- | ------- | ------- |
| 1          | q₁  | 0       | 0        | 0       | 287.87   | 0       | 1       |
| 2          | q₂  | -90     | 20.174   | -90     | 0        | -1      | 0       |
| 3          | q₃  | 0       | 260.986  | 0       | 0        | 0       | 1       |
| 4          | q₄  | 0       | 19.219   | 0       | 260.753  | 0       | 1       |
| 5          | q₅  | 90      | 0        | 0       | 0        | 1       | 0       |
| 6          | q₆  | -90     | 0        | 180     | 74.745   | -1      | 0       |

> **Sources:** [Arctos Specifications Page](https://arctosrobotics.com/docs/#specifications), URDF in [ROS repository](https://github.com/Arctos-Robotics/ROS/tree/main/arctos_urdf_description).

---

## 5. MKS Driver Configuration

### 5.1 Required Settings Per Motor

Each motor must be configured before use:

| Setting    | Value      | Menu/Command | Notes                            | Source         |
| ---------- | ---------- | ------------ | -------------------------------- | -------------- |
| Mode       | SR_vFOC    | Menu → Mode / cmd 0x82 (val 5) | Serial FOC mode, up to 3000 RPM | Arctos docs |
| CanRate    | 500K       | Menu → CanRate / cmd 0x8A (val 2) | Must match CANable              | Arctos docs |
| CanID      | 01–06      | Menu → CanID / cmd 0x8B  | Unique per motor                 | Arctos docs    |
| En         | Hold       | Menu → En / cmd 0x85 (val 2) | Always enabled (no EN pin in serial mode) | ⚠️ Recommended* |
| MPlyer     | Enable     | Menu → MPlyer / cmd 0x89 (val 1) | Internal 256 subdivision for smoothness | MKS default |
| Protect    | Enable     | Menu → Protect / cmd 0x88 (val 1) | Locked-rotor protection — see note below | ⚠️ Recommended* |

> **\*Recommended settings** are not from Arctos official docs. MKS defaults are En=L and Protect=Disable. En=Hold is recommended because serial mode doesn't use the physical EN pin. Protect=Enable is recommended for safety but may trigger false positives under heavy load or fast acceleration. Test on your hardware and disable if it causes issues.

### 5.2 Working Current Defaults

| Driver   | Default Current | Max Current | Adjustable via    |
| -------- | --------------- | ----------- | ----------------- |
| MKS 57D  | 3200 mA         | 5200 mA     | Menu Ma / cmd 0x83 |
| MKS 42D  | 1600 mA         | 3000 mA     | Menu Ma / cmd 0x83 |

### 5.3 Work Mode Comparison

| Mode     | Max RPM | Current Behavior                  | Encoder Required |
| -------- | ------- | --------------------------------- | ---------------- |
| SR_OPEN  | 400     | Fixed at Ma setting               | No               |
| SR_CLOSE | 1500    | Fixed at Ma setting               | Yes              |
| SR_vFOC  | 3000    | Self-adaptive, max limited by Ma  | Yes              |

SR_vFOC is recommended for highest speed. SR_CLOSE may be better for applications needing consistent torque at lower speeds (the MKS manual notes it is better than CR_vFOC for 3D printing).

---

## 6. Key Commands by Use Case

This section highlights the commands you'll actually use in software. The full command reference follows in Section 7.

### 6.1 Runtime — Used Constantly

| Code | Command | What It Does | When You Use It |
| ---- | ------- | ------------ | --------------- |
| 0xF5 | Absolute axis move | Move motor to a specific encoder position with speed and acceleration | Every joint movement. Supports real-time target updates while moving. |
| 0x31 | Read encoder (addition) | Returns cumulative encoder position (int48) | Position feedback. Divide by gear ratio × 16384 to get joint angle. |
| 0xF1 | Query motor status | Returns: 1=stop, 2=accel, 3=decel, 4=full speed, 5=homing, 6=cal | Check if a move is finished, especially in polling mode. |
| 0xF7 | Emergency stop | Immediately stops the motor | Safety. Wire to a keyboard shortcut or physical button from day one. |
| 0xF3 | Enable/disable motor | Energize or de-energize the motor | Enable at startup. Disable to let arm go limp for manual positioning. |

### 6.2 Startup / Homing

| Code | Command | What It Does | When You Use It |
| ---- | ------- | ------------ | --------------- |
| 0x91 | Go home | Motor runs until it hits the limit switch | Once per joint at startup to establish zero position. |
| 0x3B | Read go-home status | Returns: 0=going, 1=success, 2=fail | Poll after sending 0x91 to know when homing completes. |
| 0x92 | Set current position as zero | Defines current position as the zero point without moving | After homing, or to redefine origin for teaching. |
| 0x90 | Set home parameters | Configures homing direction, speed, trigger level, endstop limits | Once per joint during initial setup. |

### 6.3 Diagnostics / Error Handling

| Code | Command | What It Does | When You Use It |
| ---- | ------- | ------------ | --------------- |
| 0x39 | Read shaft angle error | Difference between target and actual position (51200 = 360°) | Debug accuracy issues. Large persistent error = mechanical problem. |
| 0x3E | Read protection status | Returns 1 if stall protection has triggered | Check after unexpected stops. |
| 0x3D | Release stall protection | Clears protection state so motor can move again | After investigating and resolving a stall. |
| 0x34 | Read IO port status | Bit field showing limit switch and output states | Verify endstops are working before homing. |
| 0x32 | Read speed (RPM) | Returns real-time motor speed | Monitoring and debugging. |

### 6.4 One-Time Configuration

These are typically set once via the onboard menu or during initial setup, not during normal operation:

| Code | Command | Typical Value |
| ---- | ------- | ------------- |
| 0x82 | Set work mode | 5 (SR_vFOC) |
| 0x8A | Set CAN bitrate | 2 (500K) |
| 0x8B | Set CAN ID | 0x01–0x06 per joint |
| 0x85 | Set EN active | 2 (Hold) |
| 0x80 | Calibrate encoder | Motor must be unloaded |
| 0x9E | Set limit port remap | 1 (enable) — MKS 42D only |

### 6.5 Commands You Probably Won't Need

| Code | Command | Why You Can Skip It |
| ---- | ------- | ------------------- |
| 0x30 | Read encoder (carry) | Use 0x31 instead — cumulative value is easier to work with |
| 0x33 | Read pulse count | Counts step/dir pulses, only relevant for pulse mode (CR_) |
| 0xFD | Position mode 1 (relative pulses) | Relative moves are harder to track; use 0xF5 absolute instead |
| 0xFE | Position mode 2 (absolute pulses) | Pulse-based; 0xF5 uses encoder units which is more natural |
| 0xF4 | Position mode 3 (relative axis) | Relative moves; 0xF5 absolute is safer — you always know where you're going |
| 0xF6 | Speed mode | Continuous rotation; arms rarely need this unless jogging |
| 0xFF | Save speed mode params | Only for auto-spin on power-up |
| 0x8D | Set group ID | Motors don't respond to group commands, so you can't confirm receipt |

---

## 7. CAN Communication Protocol — Full Reference

### 7.1 Frame Format

Standard CAN frames. All data is big-endian unless noted.

**Downlink (PC → Motor):**

```
CAN_ID | DLC | byte1 (code) | byte2..byte(n-1) (data) | byte(n) (CRC)
```

**Uplink (Motor → PC):**

```
CAN_ID | DLC | byte1 (code) | byte2..byte(n-1) (data) | byte(n) (CRC)
```

**CRC Calculation:**

```
CRC = (CAN_ID + byte1 + byte2 + ... + byte(n-1)) & 0xFF
```

### 7.2 Addressing

| Address | Meaning                                      |
| ------- | -------------------------------------------- |
| 0x00    | Broadcast (no response from slaves)          |
| 0x01–0x7FF | Individual slave ID                       |
| Group ID | Set via cmd 0x8D (no response from slaves) |

### 7.3 Essential Commands — Quick Reference

#### Read Commands

| Code | Command                  | Send (DLC=2)     | Response                           |
| ---- | ------------------------ | ---------------- | ---------------------------------- |
| 0x30 | Read encoder (carry)     | `ID 30 CRC`     | carry(int32) + value(uint16)       |
| 0x31 | Read encoder (addition)  | `ID 31 CRC`     | value(int48) — cumulative position |
| 0x32 | Read speed (RPM)         | `ID 32 CRC`     | speed(int16)                       |
| 0x33 | Read pulse count         | `ID 33 CRC`     | pulses(int32)                      |
| 0x34 | Read IO port status      | `ID 34 CRC`     | status(uint8) — bit0=IN1, bit1=IN2, bit2=OUT1, bit3=OUT2 |
| 0x39 | Read shaft angle error   | `ID 39 CRC`     | error(int32) — 51200 = 360°        |
| 0x3A | Read enable status       | `ID 3A CRC`     | 1=enabled, 0=disabled              |
| 0x3B | Read go-home status      | `ID 3B CRC`     | 0=going, 1=success, 2=fail         |
| 0x3E | Read protection status   | `ID 3E CRC`     | 1=protected, 0=not protected       |
| 0xF1 | Query motor status       | `ID F1 CRC`     | 0=fail, 1=stop, 2=accel, 3=decel, 4=full speed, 5=homing, 6=cal |

#### Set Commands

| Code | Command                    | Data                        | Notes                      |
| ---- | -------------------------- | --------------------------- | -------------------------- |
| 0x80 | Calibrate encoder          | `00`                        | Motor must be unloaded     |
| 0x82 | Set work mode              | `mode(0–5)`                 | 0=CR_OPEN ... 5=SR_vFOC    |
| 0x83 | Set working current        | `ma(uint16)`                | In mA                      |
| 0x84 | Set subdivision            | `micstep(0x00–0xFF)`        |                            |
| 0x85 | Set EN pin active          | `0=L, 1=H, 2=Hold`         |                            |
| 0x86 | Set direction              | `0=CW, 1=CCW`              | Pulse mode only            |
| 0x88 | Set stall protection       | `0=disable, 1=enable`       |                            |
| 0x89 | Set MPlyer interpolation   | `0=disable, 1=enable`       |                            |
| 0x8A | Set CAN bitrate            | `0=125K, 1=250K, 2=500K, 3=1M` |                        |
| 0x8B | Set CAN ID                 | `ID(uint16, 0x00–0x7FF)`   | 0 = broadcast              |
| 0x8C | Set response/active mode   | `respon(0/1), active(0/1)`  | See section 7.4            |
| 0x8D | Set group ID               | `ID(uint16, 0x01–0x7FF)`   |                            |
| 0x8F | Lock/unlock keys           | `0=unlock, 1=lock`          |                            |
| 0x9B | Set holding current %      | `0x00=10% ... 0x08=90%`     | OPEN/CLOSE mode only       |
| 0x9E | Set limit port remap       | `0=disable, 1=enable`       | 42D needs this for 2 limits|
| 0x3D | Release stall protection   | (no data)                   |                            |
| 0x3F | Restore defaults           | (no data)                   | Requires recalibration     |
| 0x41 | Restart motor              | (no data)                   |                            |

#### Home Commands

| Code | Command                     | Data                                            |
| ---- | --------------------------- | ----------------------------------------------- |
| 0x90 | Set home parameters         | `homeTrig(0/1), homeDir(0/1), homeSpeed(uint16), EndLimit(0/1)` |
| 0x91 | Go home                     | (no data) — motor runs until limit switch hit   |
| 0x92 | Set current position as zero| (no data) — software zero without moving         |
| 0x94 | Set noLimit home params     | `retValue(uint32), ma(uint16)`                  |

#### Motion Commands (requires SR_OPEN/SR_CLOSE/SR_vFOC mode)

| Code | Command                          | Data Format                                         |
| ---- | -------------------------------- | --------------------------------------------------- |
| 0xF3 | Enable/disable motor             | `0=disable, 1=enable`                               |
| 0xF6 | Speed mode                       | `dir\|speed(12bit), acc(uint8)` — speed=0 to stop    |
| 0xF7 | Emergency stop                   | (no data) — immediate stop, caution above 1000 RPM  |
| 0xFD | Position mode 1 (relative pulses)| `dir\|speed(12bit), acc(uint8), pulses(uint24)`      |
| 0xFE | Position mode 2 (absolute pulses)| `speed(uint16), acc(uint8), absPulses(int24)`        |
| 0xF4 | Position mode 3 (relative axis)  | `speed(uint16), acc(uint8), relAxis(int24)`          |
| 0xF5 | Position mode 4 (absolute axis)  | `speed(uint16), acc(uint8), absAxis(int24)`          |
| 0xFF | Save/clear speed mode params     | `0xC8=save, 0xCA=clear`                              |

---

## 8. Endstop / Homing Configuration

### 8.1 Sensor Hardware

- **KY-003** — Hall effect limit switches (6 total, one per axis)
- **WSH231** — Dual hall effect sensors (3 total, shared between axes)

### 8.2 IO Port Mapping

| Port  | Function                          | 57D | 42D |
| ----- | --------------------------------- | --- | --- |
| IN_1  | Home / left limit                 | ✓   | ✓   |
| IN_2  | Right limit                       | ✓   | ✗   |
| OUT_1 | Stall indication (0=protected)    | ✓   | ✗   |
| OUT_2 | Reserved                          | ✓   | ✗   |

### 8.3 Port Remapping (MKS 42D)

The 42D only has IN_1 natively. To use two limit switches in serial mode, enable port remapping:

```
Send: ID 9E 01 CRC
```

After remapping: IN_1 → EN port, IN_2 → DIR port. The COM port must be connected to the corresponding high-level voltage.

Per-axis remap commands:
- Z (ID 03): `03 9E 01 A2`
- A (ID 04): `04 9E 01 A3`
- B (ID 05): `05 9E 01 A4`
- C (ID 06): `06 9E 01 A5`

### 8.4 MKS 57D DIP Switch Settings

| PIN | ON                                    | OFF                           |
| --- | ------------------------------------- | ----------------------------- |
| 3   | EVCC/EGND powered by SERVO57D 5V     | Powered externally (3.3–24V)  |
| 2   | (same as pin 3)                       | (same as pin 3)               |
| 1   | CAN 120Ω termination                 | No termination                |

For endstop sensors on 57D: flip DIP switches 2 and 3 to ON.

### 8.5 Homing Modes

| Hm_Mode  | Description                                                    |
| -------- | -------------------------------------------------------------- |
| Limited  | Uses limit switch — motor runs until switch triggers (default) |
| noLimit  | Motor runs with fixed torque (Hm_Ma) until obstacle, then reverses by retValue and stops. Stop point = zero. |

---

## 9. CAN Bus Wiring

### 9.1 Topology

Daisy chain: **CANable V2 → Motor X → Motor Y → Motor Z → Motor A → Motor B → Motor C**

### 9.2 Termination

120Ω termination resistors required at **both ends** of the chain:
- CANable V2 end (built-in or add manually)
- Last motor (Motor C) — use onboard jumper/DIP switch

For single-slave testing, termination is not needed.

### 9.3 Wiring Best Practices

- Use shielded twisted pair for CAN-H and CAN-L
- Connect all GND lines together (host GND to motor GND)
- Wire motors to MKS drivers BEFORE bolting down (no space after mounting on A/B/C axes)
- Do not hot-plug motor cables or data cables

---

## 10. End Effector (TBD)

TBD

---

## 11. Common Error Messages

| Error Message             | Cause                                    | Solution                              |
| ------------------------- | ---------------------------------------- | ------------------------------------- |
| Not Cal                   | Motor not calibrated                     | Run calibration (cmd 0x80)            |
| Reverse Lookup Error!     | Calibration failed                       | Check magnet and motor shaft          |
| Magnet Loss!              | No magnet detected                       | Install magnet on shaft               |
| Magnet Strong!            | Magnet too close to encoder              | Adjust magnet distance                |
| Magnet Weak!              | Magnet too far from encoder              | Adjust magnet distance                |
| Encoder Error!            | Encoder read failure                     | Check magnet and shaft                |
| Phase Line Error!         | Motor wiring wrong or power insufficient | Check wiring order; verify 24V/1A or 12V/2A supply |
| Wrong Protect!            | Locked-rotor protection triggered        | Release via Enter button or cmd 0x3D  |

---

## 12. Safety Reminders

- **Reversed MKS driver polarity permanently destroys the board.** Triple-check before powering on.
- Always implement emergency stop (cmd 0xF7) accessible at all times in software.
- Do not stop motors immediately above 1000 RPM — use deceleration (acc ≠ 0).
- Calibrate motors unloaded only.
- Bolt the robot to a secure surface — it can tip during operation.
- Start with slow movements when testing new code.

---

## 13. Useful Links

- MKS SERVO42D/57D CAN Manual: (uploaded PDF, V1.0.5)
- Interactive Wiring Schematic: https://app.cirkitdesigner.com/project/6c301cb3-c8e1-4516-b3c2-bac0ea6c533a
- CAD Files: https://arctosrobotics.com/product/cad-files/
- BOM: https://arctosrobotics.com/bom/

For general project links (Arctos docs, Discord, GitHub), see the [README](../README.md#resources).
