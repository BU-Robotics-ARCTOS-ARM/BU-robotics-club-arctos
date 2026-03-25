"""Interactive script to test forward kinematics."""

import sys
import os

# Ensure the project root is in the path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from arctos.kinematics import Kinematics
from arctos.config import DH_PARAMS

def main():
    kin = Kinematics(DH_PARAMS)
    
    print("Arctos Forward Kinematics Test")
    print("-" * 30)
    print("Enter 6 joint angles in degrees, separated by spaces.")
    print("Example: 0 0 0 0 0 0")
    print("Type 'q' to quit.")
    
    while True:
        try:
            line = input("\nJoint angles: ").strip()
            if not line:
                continue
            if line.lower() == 'q':
                break
            
            angles = [float(x) for x in line.split()]
            if len(angles) != 6:
                print(f"Error: Expected 6 angles, got {len(angles)}")
                continue
            
            pose = kin.forward(angles)
            print("-" * 15)
            print(f"Resulting Pose:")
            print(f"  X: {pose.x:0.2f} mm")
            print(f"  Y: {pose.y:0.2f} mm")
            print(f"  Z: {pose.z:0.2f} mm")
            print(f"  Roll:  {pose.roll:0.2f} deg")
            print(f"  Pitch: {pose.pitch:0.2f} deg")
            print(f"  Yaw:   {pose.yaw:0.2f} deg")
            print("-" * 15)
            
        except ValueError:
            print("Error: Invalid input. Please enter numbers.")
        except KeyboardInterrupt:
            print("\nExiting.")
            break
        except Exception as e:
            print(f"Error: {e}")

if __name__ == "__main__":
    main()
