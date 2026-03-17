#say thanks to Claude for the code :)
import can
from mks_servo_can import MksServo
from mks_servo_can.mks_enums import Direction   # <-- Import the Direction enum
import time


# 1. Initialize the CAN bus
print("Initializing CAN bus on COM3...")
bus = can.interface.Bus(interface="slcan", channel="COM3", bitrate=500000)
notifier = can.Notifier(bus, [])

# 2. Connect to the MKS 42D (CAN ID = 1)
servo = MksServo(bus, notifier, 1)

def safe_stop():
    """Sends a zero-speed command to stop the motor."""
    try:
        servo.run_motor_in_speed_mode(Direction.CW, 0, 50)
    except Exception as stop_err:
        print(f"Warning: Could not send stop command: {stop_err}")

try:
    print("\n--- MOTOR TEST STARTING ---")
    print("Press Ctrl+C in the terminal at any time to EMERGENCY STOP.")
    time.sleep(2)

    for cycle in range(3):
        print(f"\nCycle {cycle + 1}: Spinning Forward (CW)...")
        servo.run_motor_in_speed_mode(Direction.CW, 200, 50)  # Use enum, plain int
        time.sleep(2)

        print(f"Cycle {cycle + 1}: Spinning Backward (CCW)...")
        servo.run_motor_in_speed_mode(Direction.CCW, 200, 50)
        time.sleep(2)

    print("\nFinished 3 cycles. Stopping motor.")
    safe_stop()

except KeyboardInterrupt:
    print("\n[!] EMERGENCY STOP TRIGGERED BY USER [!]")
    safe_stop()

except Exception as e:
    print(f"\nError: {e}")
    safe_stop()

finally:
    time.sleep(0.5)
    notifier.stop()
    bus.shutdown()
    print("CAN bus shut down cleanly.")