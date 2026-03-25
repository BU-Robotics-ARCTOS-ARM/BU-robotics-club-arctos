"""Single source of truth for all Arctos hardware constants.

All hardware-specific values live here. If a gear ratio, CAN ID, or DH
parameter changes, update it in this file only.

Sources:
    docs/arctos_arm_specifications.md
    docs/ARCHITECTURE.md
    Arctos DH standard convention (verified)
"""

# ---------------------------------------------------------------------------
# CAN bus settings
# ---------------------------------------------------------------------------
CAN_CHANNEL = 'can0'       # 'slcan0' on some setups — depends on OS/adapter
CAN_BITRATE = 500_000      # 500 Kbps (MKS default)

# ---------------------------------------------------------------------------
# Joint definitions
# (name, can_id, gear_ratio, min_angle_deg, max_angle_deg)
#
# Angle limits are placeholders — verify on hardware before running.
# ---------------------------------------------------------------------------
JOINTS = [
    ('X_base',     0x01,  13.5,   -180, 180),
    ('Y_shoulder', 0x02, 150.0,    -90,  90),
    ('Z_elbow',    0x03, 150.0,   -135, 135),
    ('A_wrist1',   0x04,  48.0,   -180, 180),
    ('B_wrist2',   0x05,  67.82,  -120, 120),
    ('C_wrist3',   0x06,  67.82,  -360, 360),
]

# ---------------------------------------------------------------------------
# Encoder
# ---------------------------------------------------------------------------
ENCODER_COUNTS_PER_REV = 16384  # 14-bit encoder (0x4000)

# ---------------------------------------------------------------------------
# Denavit-Hartenberg parameters (standard/classic convention)
# format: (alpha_deg, a_mm, theta_offset_deg, d_mm)
#
# These parameters are adjusted to follow standard DH where:
# alpha_1 = -90 ensures J2 (Shoulder) rotates around horizontal axis.
# ---------------------------------------------------------------------------
DH_PARAMS = [
    (-90,      0,       0,   287.87 ), # J1: Rotation around vertical Z0
    (  0,    260.986, -90,     0    ), # J2: Shoulder (Offset -90 for home pos)
    (-90,     19.219,   0,     0    ), # J3: Elbow
    ( 90,      0,       0,   260.753), # J4: Wrist 1
    (-90,      0,       0,     0    ), # J5: Wrist 2
    (  0,      0,     180,    74.745), # J6: Wrist 3
]

# ---------------------------------------------------------------------------
# Default motion parameters
# ---------------------------------------------------------------------------
DEFAULT_SPEED = 300   # RPM
DEFAULT_ACC   = 2     # 0–255
