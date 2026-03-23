# TODO

Project tasks organized by phase. Claim a task by putting your name next to it.

Legend: `[ ]` = open, `[~]` = in progress, `[x]` = done

---

## Claiming Tasks

Put your name and date next to a task when you start it:

```
- [x] Create documentations — @ben 2026-03-23
```

If you abandon a task, remove your name so someone else can pick it up.

> **Note:** For now, this TODO list is edited by committing directly to `main`. This will likely be replaced with a more elegant solution (e.g., GitHub Issues or a project board) in the future.

---

## Phase 1 — Foundation

> All tasks in this phase can be done **in parallel** (no dependencies on each other).

- [ ] Create `arctos/` package with `__init__.py` and `config.py` (CAN settings, joint definitions, DH params, constants)
- [ ] Implement `CANInterface` — open/close connection, send/receive frames, timeout handling, tests
- [ ] Implement forward kinematics — DH parameter transforms, `Kinematics.forward()`, tests *(pure math, no HW needed)*

## Phase 2 — Motor Driver

> Depends on Phase 1 (CAN interface + config). All tasks in this phase can be done **in parallel**.

- [ ] Motor read commands — `read_encoder()`, `read_speed()`, `read_status()`, CRC calc, tests
- [ ] Motor write commands — `move_to_axis()`, `set_speed()`, `stop()`, `emergency_stop()`, `enable()`, tests
- [ ] Motor homing commands — `go_home()`, `read_home_status()`, `set_zero()`, protection status/release, tests

## Phase 3 — Joint Control

> Depends on Phase 2. All tasks in this phase can be done **in parallel**.

- [ ] Joint angle/encoder math — `angle_to_encoder()`, `encoder_to_angle()`, limit enforcement, tests
- [ ] Joint motion control — `start_move()`, `wait_until_done()`, `is_moving()`, `get_angle()`, tests
- [ ] Joint homing — `start_home()` with hall effect endstop logic, tests

## Phase 4 — Arm Coordination

> Depends on Phase 3. All tasks in this phase can be done **in parallel**.

- [ ] Arm multi-joint control — `move_joints()`, `get_joint_angles()`, `enable_all()`, `is_moving()`, tests
- [ ] Arm homing — `home_all()` with safe joint ordering (Z first, then Y, then X), tests
- [ ] Arm emergency stop — `emergency_stop()` stops all motors immediately, tests

## Phase 5 — Integration

> Depends on Phase 4 + Phase 1 kinematics. Tasks can be done **in parallel**.

- [ ] Inverse kinematics — `Kinematics.inverse()`, `is_reachable()`, tests
- [ ] Utility scripts — `ping_all_motors.py`, `home_all.py`, `jog_joint.py`, `read_positions.py`, `emergency_stop.py`

## Phase 6 — UI

> Depends on Phase 5.

- [ ] Design and build basic GUI (TBD)
