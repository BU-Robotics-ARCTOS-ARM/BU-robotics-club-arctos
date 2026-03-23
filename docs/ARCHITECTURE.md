# Software Architecture

How the Arctos closed-loop software is structured, how the layers connect, and where your code belongs.

> **Note:** This architecture is a rough barebone design and is subject to change as unforeseen requirements, hardware constraints, or integration challenges arise. Expect layers, interfaces, and file organization to evolve as the project matures.

---

## Overview

The software is organized in layers. Each layer only talks to the one directly below it. This keeps things modular. You can change how CAN frames are sent without touching kinematics, or swap out the UI without rewriting motor control.

```
┌─────────────────────────────────────┐
│           User Interface            │  ← GUI,
├─────────────────────────────────────┤
│            Kinematics               │  ← FK, IK, angle ↔ cartesian
├─────────────────────────────────────┤
│          Arm Controller             │  ← coordinates 6 joints, homing
├─────────────────────────────────────┤
│          Joint Controller           │  ← single joint: angle ↔ encoder
├─────────────────────────────────────┤
│           Motor Driver              │  ← CAN commands, responses, CRC
├─────────────────────────────────────┤
│            CAN Interface            │  ← python-can / CANable hardware
└─────────────────────────────────────┘
```

### Think of it like the postal system

| Postal system | | Software layer |
|---|---|---|
| You, the sender | → | User Interface |
| Post office (translates address to a route) | → | Kinematics |
| Regional hub (dispatches to the right trucks) | → | Arm Controller |
| Truck driver (follows their single route) | → | Joint Controller |
| Road rules & signage (protocol the driver follows) | → | Motor Driver |
| The road itself (carries the truck, doesn't know what's inside) | → | CAN Interface |

Each layer only knows its own job. The road doesn't know what's in the truck, and the sender doesn't need to know which roads exist.

---

## Folder Structure

```
BU-robotics-club-arctos/
├── docs/
│   ├── arctos_arm_specifications.md
│   ├── ARCHITECTURE.md          ← this file
│   ├── DEV_QUICKSTART.md
│   └── meeting_notes/
│
├── arctos/
│   ├── __init__.py
│   ├── can_interface.py         ← Layer 1: raw CAN send/receive
│   ├── motor_driver.py          ← Layer 2: MKS command protocol
│   ├── joint.py                 ← Layer 3: single joint abstraction
│   ├── arm.py                   ← Layer 4: 6-joint coordination
│   ├── kinematics.py            ← Layer 5: FK and IK
│   └── config.py                ← joint definitions, gear ratios, CAN IDs
│
├── ui/
│   └── (GUI interface TBD)
│
├── tests/
│   ├── test_can_interface.py
│   ├── test_motor_driver.py
│   ├── test_joint.py
│   ├── test_arm.py
│   └── test_kinematics.py
│
├── scripts/
│   ├── ping_all_motors.py       ← verify all 6 motors respond
│   ├── home_all.py              ← home all joints
│   ├── jog_joint.py             ← manually move one joint
│   ├── read_positions.py        ← print all joint angles
│   └── emergency_stop.py        ← stop everything immediately
│
├── .gitignore
├── README.md
└── pyproject.toml
```

---

## Layer Details

### Layer 1: CAN Interface (`can_interface.py`)

**Purpose:** Thin wrapper around `python-can`. Sends and receives raw CAN frames. Nothing specific to MKS motors lives here.

**Key responsibilities:**
- Open/close the CAN bus connection
- Send a frame (CAN ID + data bytes)
- Receive a frame (blocking or with timeout)
- Handle connection errors (adapter unplugged, bus-off)

**Example interface:**

```python
class CANInterface:
    def __init__(self, channel='can0', bitrate=500000):
        """Open CAN bus connection."""

    def send(self, can_id: int, data: bytes) -> None:
        """Send a raw CAN frame."""

    def receive(self, timeout: float = 1.0) -> tuple[int, bytes] | None:
        """Receive a CAN frame. Returns (can_id, data) or None on timeout."""

    def close(self) -> None:
        """Close the connection."""
```

**Does NOT:** Calculate CRC, parse MKS responses, know about motors or joints.

---

### Layer 2: Motor Driver (`motor_driver.py`)

**Purpose:** Speaks the MKS CAN protocol. Knows how to build command frames, calculate CRC, parse response frames. One instance per physical motor.

**Key responsibilities:**
- Build command bytes with correct CRC
- Send commands and wait for responses
- Parse response status codes
- Handle the MKS byte encoding (direction+speed packing, signed integers)

**Example interface:**

```python
class MotorDriver:
    def __init__(self, can: CANInterface, can_id: int):
        """Create driver for one MKS motor at the given CAN ID."""

    def read_encoder(self) -> int:
        """Read cumulative encoder position (command 0x31). Returns int."""

    def read_speed(self) -> int:
        """Read current speed in RPM (command 0x32). Returns signed int."""

    def read_status(self) -> MotorStatus:
        """Query motor status (command 0xF1). Returns enum."""

    def move_to_axis(self, position: int, speed: int, acc: int) -> bool:
        """Absolute axis move (command 0xF5). Returns True if started."""

    def set_speed(self, direction: int, speed: int, acc: int) -> bool:
        """Speed mode (command 0xF6). Returns True if started."""

    def stop(self, acc: int = 0) -> bool:
        """Stop motor. acc=0 for immediate, >0 for deceleration."""

    def emergency_stop(self) -> bool:
        """Emergency stop (command 0xF7)."""

    def enable(self, enabled: bool = True) -> bool:
        """Enable or disable motor (command 0xF3)."""

    def go_home(self) -> bool:
        """Start homing (command 0x91)."""

    def read_home_status(self) -> HomeStatus:
        """Check homing progress (command 0x3B)."""

    def set_zero(self) -> bool:
        """Set current position as zero (command 0x92)."""

    def read_protection_status(self) -> bool:
        """Check if stall protection triggered (command 0x3E)."""

    def release_protection(self) -> bool:
        """Clear stall protection (command 0x3D)."""

    @staticmethod
    def calc_crc(can_id: int, data: bytes) -> int:
        """CRC = (can_id + sum(data)) & 0xFF"""
        return (can_id + sum(data)) & 0xFF
```

**Does NOT:** Know about gear ratios, joint angles, or kinematics.

---

### Layer 3: Joint Controller (`joint.py`)

**Purpose:** Represents a single joint of the arm. Converts between human-readable angles (degrees) and raw encoder counts using the gear ratio. Enforces joint limits.

**Key responsibilities:**
- Angle-to-encoder and encoder-to-angle conversion
- Joint angle limits (min/max degrees)
- Homing sequence for this specific joint
- Current angle tracking

**Example interface:**

```python
class Joint:
    def __init__(self, motor: MotorDriver, name: str, gear_ratio: float,
                 min_angle: float, max_angle: float):
        """Create joint with its motor, name, gear ratio, and limits."""

    def get_angle(self) -> float:
        """Read current joint angle in degrees."""

    def start_move(self, angle_deg: float, speed_rpm: int = 300, acc: int = 2) -> bool:
        """Send move command to target angle in degrees. Returns immediately. Enforces limits."""

    def wait_until_done(self, timeout: float = 10.0) -> bool:
        """Poll motor status until stopped. Returns False on timeout."""

    def start_home(self) -> bool:
        """Start homing sequence for this joint. Returns immediately."""

    def is_moving(self) -> bool:
        """Check if joint is currently in motion (polls motor status)."""

    def angle_to_encoder(self, angle_deg: float) -> int:
        """Convert degrees to encoder counts."""
        return int(angle_deg / 360.0 * self.gear_ratio * 16384)

    def encoder_to_angle(self, encoder: int) -> float:
        """Convert encoder counts to degrees."""
        return encoder * 360.0 / (self.gear_ratio * 16384)
```

**Does NOT:** Know about other joints, cartesian positions, or trajectories.

---

### Layer 4: Arm Controller (`arm.py`)

**Purpose:** Coordinates all 6 joints as a single robot arm. Handles multi-joint moves, homing sequence, and emergency stop for the whole arm.

**Key responsibilities:**
- Initialize and home all joints in the correct order
- Move multiple joints simultaneously
- Wait for all joints to finish before proceeding
- Emergency stop all joints
- Report all joint angles as a single array

**Example interface:**

```python
class ArmController:
    def __init__(self, can: CANInterface):
        """Create arm with all 6 joints from config."""

    def home_all(self, timeout: float = 30.0) -> bool:
        """Home all joints in safe order. Starts all, then waits. Returns False on timeout."""

    def get_joint_angles(self) -> list[float]:
        """Return current angles for all 6 joints in degrees."""

    def move_joints(self, angles: list[float], speed: int = 300, acc: int = 2,
                    timeout: float = 10.0) -> bool:
        """Send move commands to all 6 joints, then wait until all are done. Returns False on timeout."""

    def emergency_stop(self) -> None:
        """Immediately stop all motors."""

    def enable_all(self, enabled: bool = True) -> None:
        """Enable or disable all motors."""

    def is_moving(self) -> bool:
        """True if any joint is still moving."""
```

**Does NOT:** Know about cartesian positions, forward/inverse kinematics, or trajectories.

---

### Layer 5: Kinematics (`kinematics.py`)

**Purpose:** Converts between joint space (6 angles) and cartesian space (XYZ position + orientation of end effector). Uses DH parameters.

**Key responsibilities:**
- Forward kinematics: joint angles → end effector pose
- Inverse kinematics: desired pose → joint angles
- Workspace boundary checking

**Example interface:**

```python
class Kinematics:
    def __init__(self, dh_params: list):
        """Initialize with DH parameter table."""

    def forward(self, joint_angles: list[float]) -> Pose:
        """Given 6 joint angles (degrees), return end effector pose (XYZ + rotation)."""

    def inverse(self, target_pose: Pose) -> list[float] | None:
        """Given target pose, return joint angles. None if unreachable."""

    def is_reachable(self, target_pose: Pose) -> bool:
        """Check if a pose is within the work envelope."""
```

**Does NOT:** Send motor commands or handle CAN communication.

---

### Configuration (`config.py`)

**Owner:** Everyone references, systems integrator maintains
**Purpose:** Single source of truth for all hardware constants. If a gear ratio or CAN ID changes, it changes in one place only.

```python
# CAN bus settings
CAN_CHANNEL = 'can0'       # or 'slcan0', depends on OS and adapter
CAN_BITRATE = 500000

# Joint definitions: (name, can_id, gear_ratio, min_angle, max_angle)
JOINTS = [
    ('X_base',    0x01, 13.5,   -180, 180),
    ('Y_shoulder', 0x02, 150.0,  -90,  90),
    ('Z_elbow',   0x03, 150.0,  -135, 135),
    ('A_wrist1',  0x04, 48.0,   -180, 180),
    ('B_wrist2',  0x05, 67.82,  -120, 120),
    ('C_wrist3',  0x06, 67.82,  -360, 360),
]

# Encoder
ENCODER_COUNTS_PER_REV = 16384  # 0x4000

# DH parameters: (alpha_deg, a_mm, theta_offset_deg, d_mm)
DH_PARAMS = [
    (0,    0,       0,    287.87),
    (-90,  20.174, -90,   0     ),
    (0,    260.986, 0,    0     ),
    (0,    19.219,  0,    260.753),
    (90,   0,       0,    0     ),
    (-90,  0,       180,  74.745),
]

# Default motion parameters
DEFAULT_SPEED = 300   # RPM
DEFAULT_ACC = 2       # 0-255
```

> **Note:** Joint angle limits above are placeholders. Determine actual limits from your hardware and update before running.

---

## Data Flow Example

**User says: "Move end effector to position (300, 0, 400) mm"**

```
1. UI receives the command
       ↓
2. Kinematics.inverse() converts cartesian pose → 6 joint angles
       ↓
3. Arm Controller receives target angles, sends each
   joint its target
       ↓
4. Each Joint converts angle (degrees) → encoder counts
   using gear ratio
       ↓
5. Motor Driver builds the 0xF5 command frame with CRC
       ↓
6. CAN Interface sends raw bytes over the bus
       ↓
   [Hardware: MKS driver moves the motor]
       ↓
7. Motor Driver reads 0x31 response → raw encoder counts
       ↓
8. Joint converts encoder counts → angle (degrees)
       ↓
9. Arm Controller confirms all joints reached target
       ↓
10. UI reports "Move complete"
```

---

## Key Design Rules

**1. Layers only call down, never up.** `motor_driver.py` never imports from `joint.py`. If a lower layer needs to notify an upper layer, use callbacks or return values, not direct imports.

**2. All hardware constants live in `config.py`.** No magic numbers scattered through the code. If you see `16384` or `0x01` hardcoded in a file that isn't `config.py`, that's a bug.

**3. Emergency stop works at every layer.** The `ArmController.emergency_stop()` calls every joint's motor `emergency_stop()`. The UI has a button/key bound to it. There is also a standalone `scripts/emergency_stop.py` that works even if the rest of the software is crashed.

**4. Every layer has tests.** Motor driver tests can use a mock CAN interface. Kinematics tests don't need hardware at all, just math. Only integration tests need the real arm.

**5. Fail safe.** If CAN communication times out, the motor doesn't respond, or an encoder reading looks wrong, stop and report the error. Never silently ignore failures.

---

## Integration Boundaries

**Lower layers → Upper layers:** The `Joint` class is the boundary. Upper layers call `joint.start_move(angle_deg)` and `joint.get_angle()` without needing to think about encoder counts, CAN frames, or CRC.

**Controller → UI:** The `ArmController` and `Kinematics` classes. The UI calls `arm.move_joints([...])` or `kinematics.inverse(pose)`. It never constructs CAN frames or does angle-to-encoder math.

---

## Future Considerations

**Trajectory planning.** The MKS servo drivers handle acceleration and velocity profiling internally, so software-level trajectory planning is not required for basic operation. If smooth multi-waypoint paths or cartesian straight-line interpolation are needed in the future, a trajectory planner layer can be added between Kinematics and the User Interface.

**Threading.** The current design is single-threaded. Commands are sent sequentially and the motors move independently after receiving them. If the UI needs to stay responsive during long moves, or if real-time status monitoring is needed, a background thread for CAN receive (dispatching incoming frames to per-motor queues) can be added to `CANInterface` without changing the upper layers.
