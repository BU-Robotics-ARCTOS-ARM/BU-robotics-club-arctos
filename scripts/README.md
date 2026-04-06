# Scripts

All scripts must be run from the **project root** (not from inside `scripts/`).

## Dependencies

Requires `python-can` and `pyserial`:
```bash
uv add python-can pyserial
```

## motor_control.py

Interactive CLI for controlling MKS SERVO42D/57D motors over CAN bus.

**With hardware (CANable V2):**
```bash
uv run python scripts/motor_control.py --interface slcan --channel /dev/cu.usbmodemXXXX
```
Find the device path with `ls /dev/cu.usb*`.

**Without hardware (virtual bus for testing):**
```bash
uv run python scripts/motor_control.py --interface virtual
```

Ctrl+C sends emergency stop to all motors before exiting.

## test_all_commands.py

Automated non-interactive test of all MKS motor commands across all 6 motors. Reports PASS/FAIL for every command.

```bash
uv run python scripts/test_all_commands.py --interface slcan --channel /dev/cu.usbmodemXXXX
uv run python scripts/test_all_commands.py --interface virtual --channel test
```
