# ─── Motor names ───────────────────────────────────────────────────────────────

HAND_MOTOR_NAME   : str  = "gripper"
SHOULDER_MOTOR_NAME : list = ["shoulder_pitch", "shoulder_roll", "arm_yaw"]
ELBOW_MOTOR_NAME    : list = ["elbow_pitch", "forearm_yaw"]
FOREARM_MOTOR_NAME  : list = ["wrist_pitch", "wrist_roll", HAND_MOTOR_NAME]
ARM_MOTOR_NAME      : list = SHOULDER_MOTOR_NAME + ELBOW_MOTOR_NAME + FOREARM_MOTOR_NAME

ANTENNA_MOTOR_NAME  : list = ["l_antenna", "r_antenna"]
DISK_MOTOR_NAME     : list = ["neck_roll", "neck_pitch", "neck_yaw"]
HEAD_MOTOR_NAME     : list = DISK_MOTOR_NAME + ANTENNA_MOTOR_NAME

# ─── Arm geometry (meters) ─────────────────────────────────────────────────────

ORIGIN_TO_SHOULDER : float = 0.19
SHOULDER_TO_ELBOW  : float = 0.28
ELBOW_TO_WRIST     : float = 0.25

# ─── Collision ─────────────────────────────────────────────────────────────────

CAPSULE_COLLISION_RADIUS : float = 0.04
TABLE_Z_COORD            : float = -0.4
TORSO_SIZE               : float = 0.263
TORSO_COLLISION_RADIUS   : float = 0.02

# ─── Arm IDs ───────────────────────────────────────────────────────────────────

ARM_LEFT_ID  : str = "l"
ARM_RIGHT_ID : str = "r"
ARM_NAME     : str = "arm"
