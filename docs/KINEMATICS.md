# Kinematics Guide

This guide explains the current functions in `arctos/kinematics.py` and the recommended order of operations for finding possible inverse kinematics (IK) solutions for the Arctos 6-DOF robot arm.

The kinematics layer converts between:

- Joint space: six joint angles, in degrees.
- Cartesian space: end effector position and orientation as `x`, `y`, `z`, `roll`, `pitch`, and `yaw`.

The current implementation supports forward kinematics. Inverse kinematics is planned but not implemented yet.

## Files Involved

- `arctos/kinematics.py`: forward kinematics and future inverse kinematics logic.
- `arctos/config.py`: DH parameters, joint limits, gear ratios, CAN IDs, and default motion settings.
- `docs/arctos_arm_specifications.md`: hardware reference and arm dimensions.

## Coordinate And Angle Conventions

The kinematics code uses Denavit-Hartenberg (DH) parameters from `config.py`.

The DH table uses this format:

```python
(alpha_deg, a_mm, theta_offset_deg, d_mm)
```

The code assumes:

- Joint angles are provided in degrees.
- DH `alpha` and static `theta_offset` values are stored in degrees, then converted to radians during `Kinematics` initialization.
- Link lengths and offsets are in millimeters.
- End effector orientation is returned as roll, pitch, and yaw in degrees.
- Roll, pitch, and yaw are extracted using a Z-Y-X convention: yaw, then pitch, then roll.

## Current Functions

### `Pose`

```python
@dataclass
class Pose:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
```

`Pose` stores the end effector position and orientation.

Position fields are in millimeters:

- `x`
- `y`
- `z`

Orientation fields are in degrees:

- `roll`
- `pitch`
- `yaw`

### `Kinematics.__init__(dh_params)`

```python
robot = Kinematics(DH_PARAMS)
```

Creates a kinematics object from a 6-row DH parameter table.

The constructor verifies that the table has shape `(6, 4)`. This matters because the arm is modeled as a 6-DOF chain, and each row must provide:

- `alpha`
- `a`
- `theta_offset`
- `d`

It also converts static angular DH values from degrees to radians.

### `Kinematics._dh_transform(alpha, a, theta, d)`

Builds the 4x4 homogeneous transformation matrix for one joint/link using standard DH parameters.

This is an internal helper. Most code should call `forward()` instead of using this directly.

### `Kinematics._dh_end_effector_transform(T)`

Extracts a `Pose` from a final 4x4 transformation matrix.

This helper reads:

- Position from the final column of the matrix.
- Orientation from the 3x3 rotation section.

It also includes a special case for gimbal lock when pitch is close to +/- 90 degrees.

### `Kinematics.forward(joint_angles)`

```python
pose = robot.forward([0, 0, 0, 0, 0, 0])
```

Computes forward kinematics.

Input:

- A list of exactly six joint angles in degrees.

Output:

- A `Pose` containing end effector `x`, `y`, `z`, `roll`, `pitch`, and `yaw`.

Recommended use:

```python
from arctos.config import DH_PARAMS
from arctos.kinematics import Kinematics

kin = Kinematics(DH_PARAMS)

joint_angles = [0, 0, 0, 0, 0, 0]
pose = kin.forward(joint_angles)

print(pose)
```

### `inverse(target_pose)`

```python
solutions = robot.inverse(target_pose)
```

This function is not implemented yet. It currently raises:

```python
NotImplementedError("Inverse kinematics is not yet implemented.")
```

The intended purpose is to take a desired `Pose` and return one or more valid joint angle solutions that can place the end effector at that target pose.

## Recommended IK Order Of Operations

For a 6-DOF arm, IK should usually return a set of possible solutions, not just one answer. A target pose may have multiple valid joint configurations, and some poses may have no valid solution.

A practical IK flow should look like this:

1. Define the target pose.

```python
target = Pose(
    x=300.0,
    y=0.0,
    z=400.0,
    roll=0.0,
    pitch=90.0,
    yaw=0.0,
)
```

2. Convert the target pose into a 4x4 target transform.

The IK solver should convert `x`, `y`, `z`, `roll`, `pitch`, and `yaw` into a target transformation matrix. This gives the solver a single mathematical object to compare against the robot's DH chain.

3. Estimate or solve the wrist center.

For many 6-DOF arms, the first three joints mostly position the wrist center and the last three joints mostly orient the tool. If the wrist geometry allows it, subtract the final tool offset from the target pose to find the wrist center.

4. Solve candidate base, shoulder, and elbow angles.

Use the wrist center to generate possible values for:

- Joint 1: base rotation.
- Joint 2: shoulder angle.
- Joint 3: elbow angle.

There may be multiple branches, such as:

- Left/right shoulder.
- Elbow up/elbow down.
- Different base rotations if the same point can be reached from multiple wrapped angles.

5. Solve candidate wrist angles.

After choosing a candidate for joints 1-3, compute the transform from the base to joint 3. Compare that against the target transform to solve the remaining wrist orientation:

- Joint 4
- Joint 5
- Joint 6

This step can also produce multiple wrist branches, such as wrist flipped versus non-flipped.

6. Generate the full solution set.

Combine the position branches and wrist branches into full six-angle candidates:

```python
[
    [q1, q2, q3, q4, q5, q6],
    [q1, q2, q3, q4_alt, q5_alt, q6_alt],
]
```

7. Normalize angles.

Convert angles into the expected degree range. For example, depending on the joint, equivalent angles like `-180` and `180` may need consistent handling.

8. Reject invalid candidates.

Filter out any candidate that violates:

- Joint angle limits from `config.py`.
- Mechanical hard stops.
- Known self-collision regions.
- Singularity rules.
- Workspace boundaries.

9. Verify each candidate with forward kinematics.

Run each candidate back through `forward()` and compare the result to the target pose.

Recommended checks:

- Position error in millimeters.
- Orientation error in degrees or as a rotation matrix/quaternion error.
- Numerical tolerance appropriate for the hardware.

10. Rank valid solutions.

If more than one valid solution remains, rank them using practical criteria:

- Smallest movement from current joint angles.
- Avoidance of joint limits.
- Avoidance of singularities.
- Preferred elbow orientation.
- Preferred wrist orientation.
- Shortest estimated motion time.

11. Return all valid solutions, plus the recommended solution.

The safest API design is to return more than one candidate and let the caller choose or inspect them.

Example future return shape:

```python
{
    "target_pose": target,
    "recommended": [10.0, -25.0, 40.0, 0.0, 75.0, 10.0],
    "solutions": [
        [10.0, -25.0, 40.0, 0.0, 75.0, 10.0],
        [10.0, 15.0, -40.0, 180.0, -75.0, -170.0],
    ],
    "errors": [
        {"position_mm": 0.2, "orientation_deg": 0.4},
        {"position_mm": 0.3, "orientation_deg": 0.6},
    ],
}
```

## Typical Usage Flow With The Arm

For real hardware, the normal motion sequence should be:

1. Initialize CAN communication.
2. Create the arm controller.
3. Enable the motors.
4. Home all joints.
5. Read the current joint angles.
6. Use `forward()` to confirm the current end effector pose.
7. Build the desired target `Pose`.
8. Run future `inverse()` to get candidate joint solutions.
9. Filter and rank the candidate solutions.
10. Move the arm to the selected joint solution with `arm.move_joints(...)`.
11. Read the final joint angles.
12. Run `forward()` again to confirm final pose.

Example intended flow:

```python
from arctos.arm import ArmController
from arctos.can_interface import CANInterface
from arctos.config import DH_PARAMS
from arctos.kinematics import Kinematics, Pose

can = CANInterface(channel="can0", bitrate=500000)
arm = ArmController(can)
kin = Kinematics(DH_PARAMS)

arm.enable_all()
arm.home_all()

current_angles = arm.get_joint_angles()
current_pose = kin.forward(current_angles)

target_pose = Pose(
    x=300.0,
    y=0.0,
    z=400.0,
    roll=0.0,
    pitch=90.0,
    yaw=0.0,
)

# Future behavior, once inverse() is implemented:
ik_result = kin.inverse(target_pose)
target_angles = ik_result["recommended"]

arm.move_joints(target_angles, speed=300, acc=2)

final_angles = arm.get_joint_angles()
final_pose = kin.forward(final_angles)
```

## Recommended IK Implementation Strategy

There are two realistic paths for implementing IK.

### Option 1: Analytical IK

Analytical IK solves the geometry directly.

Advantages:

- Fast.
- Can return distinct solution branches.
- Good for real-time control if the geometry is well understood.

Challenges:

- Harder to derive correctly.
- Depends heavily on the exact arm geometry.
- More sensitive to DH convention mistakes.
- Wrist offsets and non-ideal mechanical geometry can complicate the math.

### Option 2: Numerical IK

Numerical IK starts with one or more initial guesses and iteratively reduces pose error.

Common methods:

- Jacobian transpose.
- Pseudoinverse Jacobian.
- Damped least squares.
- Nonlinear least squares optimization.

Advantages:

- Easier to adapt to the actual DH model.
- Can work even when the geometry does not have a clean analytical solution.
- Usually faster to prototype.

Challenges:

- May converge to only one nearby solution.
- Can get stuck in local minima.
- Needs careful handling near singularities.
- Needs multiple seed guesses to find multiple possible solutions.

For this project, a good practical first version is numerical IK with multiple seed guesses, followed by forward-kinematics verification and joint-limit filtering. Analytical IK can be added later if speed or branch completeness becomes important.

## Suggested Return Contract For `inverse()`

The current type hint says:

```python
def inverse(self, target_pose: Pose) -> dict[str, list[float]]:
```

That should probably be changed before implementation. A better return value would include all valid solutions and a recommended one.

Recommended shape:

```python
def inverse(
    self,
    target_pose: Pose,
    current_angles: list[float] | None = None,
) -> dict:
    ...
```

Recommended dictionary keys:

- `solutions`: list of valid six-angle solutions.
- `recommended`: the best solution after ranking, or `None`.
- `reachable`: `True` if at least one solution was found.
- `reason`: short explanation when no solution is found.
- `errors`: pose error for each returned solution.

## Limitations

Current limitations:

- `inverse()` is not implemented yet.
- There are no kinematics unit tests yet.
- The DH parameters must be verified against the physical arm.
- Joint angle limits in `config.py` are marked as placeholders.
- `forward()` assumes exactly six joints.
- `forward()` assumes the DH convention used in `config.py` matches the physical arm and joint zero positions.
- Orientation extraction uses roll, pitch, yaw, which can become ambiguous near gimbal lock.
- `_dh_end_effector_transform()` duplicates orientation extraction logic that is also written inside `forward()`.
- Collision checking is not implemented.
- Workspace checking is not implemented.
- Singularity detection is not implemented.
- Tool/end-effector geometry beyond the DH table is not separately modeled.
- Motion planning is not implemented; the current layer only computes poses and angles.
- Encoder backlash, flex, missed steps, calibration errors, and real-world tolerances are not modeled.

Expected IK limitations even after implementation:

- Some target poses will be unreachable.
- Some reachable positions may have impossible orientations.
- Multiple valid joint solutions may exist for the same pose.
- Numerical IK may fail depending on seed guesses.
- Near singularities, small pose changes can require large joint movements.
- The best mathematical solution may not be the safest physical motion.
- Returned solutions must always be checked against joint limits and hardware constraints before motion.

## Safety Notes

Before sending IK results to the real robot:

- Verify the candidate with `forward()`.
- Confirm all joint angles are within real mechanical limits.
- Start with slow speed and low acceleration.
- Test new IK behavior in simulation or with motors disabled when possible.
- Keep emergency stop available.
- Avoid moving directly between distant IK solutions without considering the path between them.
