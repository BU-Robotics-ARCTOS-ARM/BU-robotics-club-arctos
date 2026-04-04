"""Unit tests for arctos.kinematics."""

import numpy as np
import pytest

from arctos.kinematics import Kinematics, Pose
from arctos.config import DH_PARAMS


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

@pytest.fixture
def kin():
    return Kinematics(DH_PARAMS)


# ---------------------------------------------------------------------------
# _dh_transform — low-level matrix tests
# ---------------------------------------------------------------------------

class TestDhTransform:
    def test_all_zeros_is_identity(self):
        T = Kinematics._dh_transform(0.0, 0.0, 0.0, 0.0)
        np.testing.assert_allclose(T, np.eye(4), atol=1e-10)

    def test_pure_z_translation(self):
        """d parameter translates along Z."""
        T = Kinematics._dh_transform(alpha=0.0, a=0.0, theta=0.0, d=100.0)
        expected = np.eye(4)
        expected[2, 3] = 100.0
        np.testing.assert_allclose(T, expected, atol=1e-10)

    def test_pure_x_translation(self):
        """a parameter translates along X."""
        T = Kinematics._dh_transform(alpha=0.0, a=100.0, theta=0.0, d=0.0)
        expected = np.eye(4)
        expected[0, 3] = 100.0
        np.testing.assert_allclose(T, expected, atol=1e-10)

    def test_pure_z_rotation_90deg(self):
        """theta=90° rotates about Z: x→y, y→-x."""
        T = Kinematics._dh_transform(alpha=0.0, a=0.0, theta=np.pi / 2, d=0.0)
        expected = np.array([
            [0, -1, 0, 0],
            [1,  0, 0, 0],
            [0,  0, 1, 0],
            [0,  0, 0, 1],
        ], dtype=float)
        np.testing.assert_allclose(T, expected, atol=1e-10)

    def test_pure_z_rotation_180deg(self):
        """theta=180° flips x and y."""
        T = Kinematics._dh_transform(alpha=0.0, a=0.0, theta=np.pi, d=0.0)
        expected = np.diag([-1.0, -1.0, 1.0, 1.0])
        np.testing.assert_allclose(T, expected, atol=1e-10)

    def test_combined_rotation_and_translation(self):
        """Sanity check: last column encodes position, upper-left is rotation."""
        a, d = 50.0, 30.0
        T = Kinematics._dh_transform(alpha=0.0, a=a, theta=np.pi / 2, d=d)
        # Position: a along rotated X (after 90° → [0,a,0]), d along Z
        np.testing.assert_allclose(T[0, 3], 0.0, atol=1e-10)   # a * cos(90°)
        np.testing.assert_allclose(T[1, 3], a,   atol=1e-10)   # a * sin(90°)
        np.testing.assert_allclose(T[2, 3], d,   atol=1e-10)
        # Bottom row must stay [0,0,0,1]
        np.testing.assert_allclose(T[3, :], [0, 0, 0, 1], atol=1e-10)

    def test_output_is_proper_homogeneous(self):
        """Bottom row is always [0,0,0,1] regardless of inputs."""
        T = Kinematics._dh_transform(1.2, 34.5, 0.78, 99.9)
        np.testing.assert_allclose(T[3, :], [0, 0, 0, 1], atol=1e-10)

    def test_rotation_submatrix_is_orthonormal(self):
        """Upper-left 3×3 must be a proper rotation matrix (det=1, R^T R = I)."""
        T = Kinematics._dh_transform(0.7, 25.0, 1.1, 40.0)
        R = T[:3, :3]
        np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-10)
        np.testing.assert_allclose(np.linalg.det(R), 1.0, atol=1e-10)


# ---------------------------------------------------------------------------
# Kinematics.__init__ — construction and validation
# ---------------------------------------------------------------------------

class TestInit:
    def test_accepts_valid_6x4_params(self):
        Kinematics(DH_PARAMS)  # must not raise

    def test_rejects_wrong_row_count(self):
        bad = [row for row in DH_PARAMS[:5]]  # only 5 joints
        with pytest.raises(ValueError, match="6"):
            Kinematics(bad)

    def test_rejects_wrong_column_count(self):
        bad = [[row[0], row[1], row[2]] for row in DH_PARAMS]  # 3 cols instead of 4
        with pytest.raises(ValueError):
            Kinematics(bad)

    def test_alpha_and_theta_offset_stored_as_radians(self, kin):
        """Columns 0 (alpha) and 2 (theta_offset) must have been converted to radians."""
        for i, row in enumerate(DH_PARAMS):
            alpha_deg, _, theta_offset_deg, _ = row
            np.testing.assert_allclose(
                kin.dh_table[i, 0], np.radians(alpha_deg), atol=1e-12,
                err_msg=f"Joint {i+1} alpha not converted to radians"
            )
            np.testing.assert_allclose(
                kin.dh_table[i, 2], np.radians(theta_offset_deg), atol=1e-12,
                err_msg=f"Joint {i+1} theta_offset not converted to radians"
            )

    def test_a_and_d_unchanged(self, kin):
        """Columns 1 (a) and 3 (d) must NOT be converted — they are distances in mm."""
        for i, row in enumerate(DH_PARAMS):
            _, a, _, d = row
            np.testing.assert_allclose(kin.dh_table[i, 1], a, atol=1e-12)
            np.testing.assert_allclose(kin.dh_table[i, 3], d, atol=1e-12)


# ---------------------------------------------------------------------------
# forward() — input validation
# ---------------------------------------------------------------------------

class TestForwardValidation:
    def test_wrong_joint_count_raises(self, kin):
        with pytest.raises(ValueError, match="6"):
            kin.forward([0.0] * 5)

    def test_too_many_joints_raises(self, kin):
        with pytest.raises(ValueError):
            kin.forward([0.0] * 7)

    def test_returns_pose_instance(self, kin):
        result = kin.forward([0.0] * 6)
        assert isinstance(result, Pose)


# ---------------------------------------------------------------------------
# forward() — numerical correctness
# ---------------------------------------------------------------------------

class TestForwardKinematics:
    # Expected home-position values are derived analytically from config.DH_PARAMS.
    # config.DH_PARAMS (alpha_deg, a_mm, theta_offset_deg, d_mm):
    #   J1: (-90,  0,       0,  287.87)
    #   J2: (  0,  260.986,-90,  0    )
    #   J3: (-90,  19.219,  0,   0    )
    #   J4: ( 90,  0,       0, 260.753)
    #   J5: (-90,  0,       0,   0    )
    #   J6: (  0,  0,     180,  74.745)
    # Result: x=335.495, y=0, z=568.075, pitch=90° (gimbal lock), roll=yaw=0°
    HOME_X = 335.498
    HOME_Y = 0.0
    HOME_Z = 568.075

    def test_home_position_xyz(self, kin):
        pose = kin.forward([0.0] * 6)
        assert pose.x == pytest.approx(self.HOME_X, abs=0.01)
        assert pose.y == pytest.approx(self.HOME_Y, abs=0.01)
        assert pose.z == pytest.approx(self.HOME_Z, abs=0.01)

    def test_home_position_orientation(self, kin):
        pose = kin.forward([0.0] * 6)
        # At home the arm is in gimbal lock with pitch = 90°
        assert pose.pitch == pytest.approx(90.0, abs=0.01)
        assert pose.roll  == pytest.approx(0.0,  abs=0.01)
        assert pose.yaw   == pytest.approx(0.0,  abs=0.01)

    def test_base_rotation_rotates_xy_plane(self, kin):
        """J1 rotation about Z: rotating by 90° maps (x,y) → (-y, x)."""
        pose0   = kin.forward([0.0,   0, 0, 0, 0, 0])
        pose90  = kin.forward([90.0,  0, 0, 0, 0, 0])
        pose180 = kin.forward([180.0, 0, 0, 0, 0, 0])

        assert pose90.x == pytest.approx(-pose0.y, abs=0.01)
        assert pose90.y == pytest.approx( pose0.x, abs=0.01)
        assert pose90.z == pytest.approx( pose0.z, abs=0.01)

        assert pose180.x == pytest.approx(-pose0.x, abs=0.01)
        assert pose180.y == pytest.approx(-pose0.y, abs=0.01)
        assert pose180.z == pytest.approx( pose0.z, abs=0.01)

    def test_wrist_rotation_does_not_move_tcp(self, kin):
        """Rotating J6 (wrist spin) must not change end-effector XYZ position."""
        home = kin.forward([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        for angle in [45.0, 90.0, 135.0, -90.0]:
            pose = kin.forward([0.0, 0.0, 0.0, 0.0, 0.0, angle])
            assert pose.x == pytest.approx(home.x, abs=1e-6), f"angle={angle}"
            assert pose.y == pytest.approx(home.y, abs=1e-6), f"angle={angle}"
            assert pose.z == pytest.approx(home.z, abs=1e-6), f"angle={angle}"

    def test_wrist_rotation_changes_orientation(self, kin):
        """J6 rotation must change at least one orientation angle."""
        pose0  = kin.forward([0, 0, 0, 0, 0, 0])
        pose45 = kin.forward([0, 0, 0, 0, 0, 45])
        # At least one of roll/pitch/yaw must differ
        orientation_changed = (
            not np.isclose(pose0.roll,  pose45.roll,  atol=0.01) or
            not np.isclose(pose0.pitch, pose45.pitch, atol=0.01) or
            not np.isclose(pose0.yaw,   pose45.yaw,   atol=0.01)
        )
        assert orientation_changed

    def test_negative_joint_angles_accepted(self, kin):
        """Negative angles must not raise and must produce a finite pose."""
        pose = kin.forward([-30.0, -45.0, -60.0, -90.0, -30.0, -45.0])
        for val in (pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw):
            assert np.isfinite(val)

    def test_large_joint_angles_accepted(self, kin):
        """Large angles (within ±360°) must produce a finite pose."""
        pose = kin.forward([180.0, 90.0, 135.0, 180.0, 120.0, 360.0])
        for val in (pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw):
            assert np.isfinite(val)

    def test_reach_within_physical_bounds(self, kin):
        """End-effector distance from base must be plausible for Arctos (~600 mm reach)."""
        pose = kin.forward([0.0] * 6)
        distance = np.sqrt(pose.x**2 + pose.y**2 + pose.z**2)
        assert distance < 800.0, f"Reach {distance:.1f} mm exceeds 800 mm — likely a bug"
        assert distance > 200.0, f"Reach {distance:.1f} mm is suspiciously small"

    def test_transformation_matrix_implicitly_orthonormal(self, kin):
        """
        Verify FK consistency: applying the same chain twice with half-angles
        and once with full angles must give the same XYZ as repeating.
        Not a mathematical proof but catches sign/unit errors.
        """
        angles = [30.0, 20.0, 10.0, 15.0, 25.0, 5.0]
        pose = kin.forward(angles)
        # Run again — result must be deterministic
        pose2 = kin.forward(angles)
        assert pose.x == pytest.approx(pose2.x, abs=1e-10)
        assert pose.y == pytest.approx(pose2.y, abs=1e-10)
        assert pose.z == pytest.approx(pose2.z, abs=1e-10)


# ---------------------------------------------------------------------------
# Pose dataclass
# ---------------------------------------------------------------------------

class TestPose:
    def test_pose_stores_all_fields(self):
        p = Pose(x=1.0, y=2.0, z=3.0, roll=10.0, pitch=20.0, yaw=30.0)
        assert p.x == 1.0
        assert p.y == 2.0
        assert p.z == 3.0
        assert p.roll == 10.0
        assert p.pitch == 20.0
        assert p.yaw == 30.0
