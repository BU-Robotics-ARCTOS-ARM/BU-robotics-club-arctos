"""Layer 5: Forward and inverse kinematics using DH parameters."""

import numpy as np
from dataclasses import dataclass

@dataclass
class Pose:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float

class Kinematics:
    def __init__(self, dh_params: list[list[float]]):
        self.dh_table = np.array(dh_params, dtype=float)
        
        # Convert static alpha and theta offset to radians
        self.dh_table[:, 0] = np.radians(self.dh_table[:, 0]) 
        self.dh_table[:, 2] = np.radians(self.dh_table[:, 2]) 

    def _dh_transform(self, alpha: float, a: float, theta: float, d: float) -> np.ndarray:
        """Helper function to calculate the 4x4 transformation matrix for a single link."""
        cos_t = np.cos(theta)
        sin_t = np.sin(theta)
        cos_a = np.cos(alpha)
        sin_a = np.sin(alpha)

        return np.array([
            [cos_t, -sin_t * cos_a,  sin_t * sin_a, a * cos_t],
            [sin_t,  cos_t * cos_a, -cos_t * sin_a, a * sin_t],
            [0,      sin_a,          cos_a,         d        ],
            [0,      0,              0,             1        ]
        ])

    def forward(self, joint_angles: list[float]) -> Pose:
        """Given 6 joint angles (degrees), return end effector pose (XYZ + rotation)."""
        if len(joint_angles) != 6:
            raise ValueError("Expected exactly 6 joint angles.")

        # 1. Convert input joint angles to radians
        q = np.radians(joint_angles)

        # 2. Initialize the base transformation matrix as an Identity Matrix
        T_final = np.eye(4)

        # 3. Iterate through all 6 joints to calculate the forward kinematics
        for i in range(6):
            alpha = self.dh_table[i, 0]
            a = self.dh_table[i, 1]
            
            # Total theta is the static offset + the dynamic joint angle
            theta = self.dh_table[i, 2] + q[i]
            
            d = self.dh_table[i, 3]

            # Get the matrix for this specific link
            T_link = self._dh_transform(alpha, a, theta, d)

            # Chain the matrices together
            T_final = T_final @ T_link

        # 4. Extract XYZ position (First three rows of the last column)
        x = T_final[0, 3]
        y = T_final[1, 3]
        z = T_final[2, 3]

        # 5. Extract Roll, Pitch, Yaw from the 3x3 rotation matrix
        # Note: This assumes a standard Z-Y-X rotation sequence (Yaw, Pitch, Roll)
        pitch = np.arctan2(-T_final[2, 0], np.sqrt(T_final[0, 0]**2 + T_final[1, 0]**2))
        
        # Handle potential Gimbal Lock (pitch near +/- 90 degrees)
        if np.isclose(np.abs(pitch), np.pi / 2):
            yaw = 0.0
            roll = np.arctan2(T_final[0, 1], T_final[1, 1])
            if pitch < 0:
                roll = -roll
        else:
            yaw = np.arctan2(T_final[1, 0], T_final[0, 0])
            roll = np.arctan2(T_final[2, 1], T_final[2, 2])

        # Return the Pose, converting angles back to degrees for readability
        return Pose(
            x=x, 
            y=y, 
            z=z,
            roll=np.degrees(roll),
            pitch=np.degrees(pitch),
            yaw=np.degrees(yaw)
        )

# ==========================================
# Testing the Logic
# ==========================================

robot_dh_parameters = [
    [  0,       0,   0, 287.87 ],  
    [-90,  20.174, -90,      0 ],  
    [  0, 260.986,   0,      0 ],  
    [  0,  19.219,   0, 260.753],  
    [ 90,       0,   0,      0 ],  
    [-90,       0, 180,  74.745]   
]

robot = Kinematics(robot_dh_parameters)

# Test with the robot in its "Zero" or "Home" position
home_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
end_effector_pose = robot.forward(home_angles)

print("End Effector Pose at Home Position:")
print(f"X: {end_effector_pose.x:.3f} mm")
print(f"Y: {end_effector_pose.y:.3f} mm")
print(f"Z: {end_effector_pose.z:.3f} mm")
print(f"Roll: {end_effector_pose.roll:.3f}°")
print(f"Pitch: {end_effector_pose.pitch:.3f}°")
print(f"Yaw: {end_effector_pose.yaw:.3f}°")
