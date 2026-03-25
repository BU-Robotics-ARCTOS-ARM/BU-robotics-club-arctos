import unittest
import numpy as np
from arctos.kinematics import Kinematics, Pose
from arctos.config import DH_PARAMS

class TestKinematics(unittest.TestCase):
    def setUp(self):
        self.kin = Kinematics(DH_PARAMS)

    def test_forward_zero_position(self):
        """
        Test forward kinematics with all joints at zero degrees.
        Based on DH parameters in config.py:
        J1: d=287.87
        J2: a=20.174, offset=-90
        J3: a=260.986
        J4: a=19.219, d=260.753
        J5: 
        J6: offset=180, d=74.745
        """
        zero_angles = [0.0] * 6
        pose = self.kin.forward(zero_angles)
        
        # We saw these values in the previous run:
        # Pose(x=260.75, y=-300.38, z=362.62, roll=-90.00, pitch=-0.00, yaw=90.00)
        self.assertAlmostEqual(pose.x, 260.75, places=1)
        self.assertAlmostEqual(pose.y, -300.38, places=1)
        self.assertAlmostEqual(pose.z, 362.62, places=1)

    def test_base_rotation(self):
        """Rotating J1 should rotate the pose around the Z axis."""
        angles0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        angles90 = [90.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        pose0 = self.kin.forward(angles0)
        pose90 = self.kin.forward(angles90)
        
        # In a 90 deg rotation around Z:
        # new_x = -old_y
        # new_y = old_x
        # new_z = old_z
        self.assertAlmostEqual(pose90.x, -pose0.y, places=2)
        self.assertAlmostEqual(pose90.y, pose0.x, places=2)
        self.assertAlmostEqual(pose90.z, pose0.z, places=2)

    def test_wrist_rotation_invariance(self):
        """Rotating the last joint (J6) should change orientation but not XYZ position."""
        angles_a = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        angles_b = [0.0, 0.0, 0.0, 0.0, 0.0, 45.0]
        
        pose_a = self.kin.forward(angles_a)
        pose_b = self.kin.forward(angles_b)
        
        # XYZ should be identical
        self.assertAlmostEqual(pose_a.x, pose_b.x, places=4)
        self.assertAlmostEqual(pose_a.y, pose_b.y, places=4)
        self.assertAlmostEqual(pose_a.z, pose_b.z, places=4)
        
        # Orientation should be different (J6 affects yaw in this configuration)
        self.assertNotAlmostEqual(pose_a.yaw, pose_b.yaw)

    def test_arm_extension(self):
        """
        Test max reach. If we align the arm horizontally.
        The reach is roughly the sum of 'a' and 'd' parameters in the extension direction.
        """
        # Align J2, J3, J4 to "stretch" the arm.
        # This depends heavily on the specific offsets.
        # Arctos reach is specified as 600mm.
        
        # Let's find an angle set that extends it.
        # J2 offset is -90. If we set J2=90, it might be horizontal.
        angles = [0.0, 90.0, 0.0, 0.0, 0.0, 0.0]
        pose = self.kin.forward(angles)
        
        distance = np.sqrt(pose.x**2 + pose.y**2 + pose.z**2)
        print(f"Extended reach (at [0,90,0,0,0,0]): {distance:.2f} mm")
        self.assertLess(distance, 700) # Should be within reasonable bounds
        self.assertGreater(distance, 300)

    def test_dh_matrix_unit(self):
        """Pure translation check."""
        # theta=0, d=100, a=0, alpha=0 -> Should be a translation of 100 in Z
        matrix = Kinematics.get_dh_matrix(0, 100, 0, 0)
        expected = np.eye(4)
        expected[2, 3] = 100
        np.testing.assert_allclose(matrix, expected, atol=1e-7)

        # theta=0, d=0, a=100, alpha=0 -> Should be a translation of 100 in X
        matrix = Kinematics.get_dh_matrix(0, 0, 100, 0)
        expected = np.eye(4)
        expected[0, 3] = 100
        np.testing.assert_allclose(matrix, expected, atol=1e-7)

if __name__ == "__main__":
    unittest.main()
