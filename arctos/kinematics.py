"""Layer 5: Forward and inverse kinematics using DH parameters."""

import numpy as np
from dataclasses import dataclass
from typing import List, Optional


@dataclass
class Pose:
    """End effector pose in cartesian space (XYZ + orientation)."""
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float

    def __repr__(self) -> str:
        return (f"Pose(x={self.x:0.2f}, y={self.y:0.2f}, z={self.z:0.2f}, "
                f"roll={self.roll:0.2f}, pitch={self.pitch:0.2f}, yaw={self.yaw:0.2f})")


class Kinematics:
    """Calculates forward and inverse kinematics using DH parameters."""

    def __init__(self, dh_params: List[tuple]):
        """
        Initialize with DH parameter table.
        Each row is (alpha_deg, a_mm, theta_offset_deg, d_mm).
        """
        self.dh_params = dh_params

    @staticmethod
    def get_dh_matrix(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
        """
        Calculates the transformation matrix for a single link using standard DH.
        theta: joint angle (radians)
        d: link offset (mm)
        a: link length (mm)
        alpha: link twist (radians)
        """
        c_theta = np.cos(theta)
        s_theta = np.sin(theta)
        c_alpha = np.cos(alpha)
        s_alpha = np.sin(alpha)

        return np.array([
            [c_theta, -s_theta * c_alpha,  s_theta * s_alpha, a * c_theta],
            [s_theta,  c_theta * c_alpha, -c_theta * s_alpha, a * s_theta],
            [0,        s_alpha,            c_alpha,           d],
            [0,        0,                  0,                 1]
        ])

    def get_all_joint_transforms(self, joint_angles: List[float]) -> List[np.ndarray]:
        """
        Calculates the transformation matrices for all joints.
        Returns a list of 4x4 matrices relative to the base.
        """
        if len(joint_angles) != len(self.dh_params):
            raise ValueError(f"Expected {len(self.dh_params)} angles, got {len(joint_angles)}")

        t_total = np.eye(4)
        transforms = [t_total.copy()]

        for angle_deg, (alpha_deg, a, theta_offset_deg, d) in zip(joint_angles, self.dh_params):
            theta = np.radians(angle_deg + theta_offset_deg)
            alpha = np.radians(alpha_deg)
            
            t_link = self.get_dh_matrix(theta, d, a, alpha)
            t_total = t_total @ t_link
            transforms.append(t_total.copy())

        return transforms

    def forward(self, joint_angles: List[float]) -> Pose:
        """
        Calculates the end effector pose given 6 joint angles in degrees.
        """
        if len(joint_angles) != len(self.dh_params):
            raise ValueError(f"Expected {len(self.dh_params)} angles, got {len(joint_angles)}")

        t_total = np.eye(4)

        for angle_deg, (alpha_deg, a, theta_offset_deg, d) in zip(joint_angles, self.dh_params):
            theta = np.radians(angle_deg + theta_offset_deg)
            alpha = np.radians(alpha_deg)
            
            t_link = self.get_dh_matrix(theta, d, a, alpha)
            t_total = t_total @ t_link

        # Extract position
        x, y, z = t_total[0:3, 3]

        # Extract orientation (Euler angles - Roll, Pitch, Yaw)
        # Using ZYX convention for yaw-pitch-roll
        sy = np.sqrt(t_total[0, 0]**2 + t_total[1, 0]**2)
        singular = sy < 1e-6

        if not singular:
            roll = np.arctan2(t_total[2, 1], t_total[2, 2])
            pitch = np.arctan2(-t_total[2, 0], sy)
            yaw = np.arctan2(t_total[1, 0], t_total[0, 0])
        else:
            roll = np.arctan2(-t_total[1, 2], t_total[1, 1])
            pitch = np.arctan2(-t_total[2, 0], sy)
            yaw = 0

        return Pose(
            x=float(x), y=float(y), z=float(z),
            roll=float(np.degrees(roll)),
            pitch=float(np.degrees(pitch)),
            yaw=float(np.degrees(yaw))
        )

    def inverse(self, target_pose: Pose) -> Optional[List[float]]:
        """
        Given target pose, return joint angles.
        TODO: Implement inverse kinematics.
        """
        raise NotImplementedError("Inverse kinematics is not yet implemented.")

    def is_reachable(self, target_pose: Pose) -> bool:
        """Check if a pose is within the work envelope."""
        # Simple placeholder for workspace boundary checking
        try:
            angles = self.inverse(target_pose)
            return angles is not None
        except NotImplementedError:
            return True # Assume reachable for now
