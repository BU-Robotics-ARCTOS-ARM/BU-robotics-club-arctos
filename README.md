# BU-robotics-club-arctos

Control software for the [Arctos](https://arctosrobotics.com/) 6-axis 3D-printed robotic arm using MKS SERVO42D/57D closed-loop drivers over CAN bus.

Built by BU Robotics Club

---

## What This Does

- Communicates with 6 MKS servo motors over CAN bus via a CANable V2 adapter
- Homes all joints using hall effect endstops
- Moves individual joints or all joints simultaneously to target angles
- Forward and inverse kinematics for cartesian positioning
- Emergency stop at every level

## Hardware

| Component | Details |
| --------- | ------- |
| Arm | Arctos v2.9.x, closed-loop version |
| Motors | 2× NEMA 23 (MKS 57D), 4× NEMA 17 (MKS 42D) |
| Interface | CANable V2 (USB-to-CAN) |
| Endstops | KY-003 + WSH231 hall effect sensors |
| End Effector | TBD |

---

## Quick Start

```bash
# Clone the repo
git clone https://github.com/hben09/BU-robotics-club-arctos.git
cd BU-robotics-club-arctos

# Install dependencies
uv sync

# Verify all motors respond
python scripts/ping_all_motors.py

# Home all joints
python scripts/home_all.py

# Read current joint positions
python scripts/read_positions.py
```

For dev setup instructions, see **[DEV_QUICKSTART.md](docs/DEV_QUICKSTART.md)**.

---

## Documentation

| Doc | What It Covers |
| --- | -------------- |
| [DEV_QUICKSTART.md](docs/DEV_QUICKSTART.md) | Environment setup, dependencies, editor configuration |
| [ARCHITECTURE.md](docs/ARCHITECTURE.md) | Software layers, folder structure, data flow |
| [arctos_arm_specifications.md](docs/arctos_arm_specifications.md) | Hardware reference — CAN commands, gear ratios, DH parameters, wiring |
| [CONTRIBUTING.md](docs/CONTRIBUTING.md) | Git workflow, code style, pull requests, testing |
| [TODO.md](docs/TODO.md) | Task board — claim a task and get building |

---

## Project Structure

See [ARCHITECTURE.md](docs/ARCHITECTURE.md) for folder layout and layer details.

---

## Usage Examples

### Move a Single Joint

```python
from arctos.arm import ArmController
from arctos.can_interface import CANInterface

can = CANInterface(channel='can0', bitrate=500000)
arm = ArmController(can)

arm.enable_all()
arm.home_all()

# Move base joint to 45 degrees
arm.joints[0].move_to(45.0, speed_rpm=300, acc=2)
```

### Move All Joints

```python
# Move all 6 joints to target angles (degrees)
arm.move_joints([45, -30, 60, 0, 90, 0], speed=300, acc=2)
```

### Get End Effector Position

```python
from arctos.kinematics import Kinematics
from arctos.config import DH_PARAMS

kin = Kinematics(DH_PARAMS)
angles = arm.get_joint_angles()
pose = kin.forward(angles)
print(f"End effector at: x={pose.x:.1f}, y={pose.y:.1f}, z={pose.z:.1f} mm")
```

### Emergency Stop

```python
arm.emergency_stop()  # stops all motors immediately
```

Or from the command line:

```bash
python scripts/emergency_stop.py
```

---

## Running Tests

```bash
# All tests
pytest tests/

# Specific layer
pytest tests/test_motor_driver.py

# Verbose output
pytest tests/ -v
```

Hardware tests are in `scripts/` and require the physical arm to be connected and powered.

---

## Contributing

See [CONTRIBUTING.md](docs/CONTRIBUTING.md) for git workflow, code style, and pull request process.

Check [TODO.md](docs/TODO.md) for available tasks.

---

## Resources

- [Arctos Documentation](https://arctosrobotics.com/docs/)
- [Arctos Discord](https://discord.com/invite/VY4c9QE5En)
- [Arctos GitHub](https://github.com/Arctos-Robotics)
- [MKS SERVO42D/57D CAN Manual (V1.0.5)](docs/arctos_arm_specifications.md#13-useful-links)

