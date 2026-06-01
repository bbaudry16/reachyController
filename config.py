# ─── Motor names ───────────────────────────────────────────────────────────────

HAND_MOTOR_NAME   : str  = "gripper"
SHOULDER_MOTOR_NAME : list = ["shoulder_pitch", "shoulder_roll", "arm_yaw"]
ELBOW_MOTOR_NAME    : list = ["elbow_pitch", "forearm_yaw"]
FOREARM_MOTOR_NAME  : list = ["wrist_pitch", "wrist_roll", HAND_MOTOR_NAME]
ARM_MOTOR_NAME      : list = SHOULDER_MOTOR_NAME + ELBOW_MOTOR_NAME + FOREARM_MOTOR_NAME

ANTENNA_MOTOR_NAME  : list = ["l_antenna", "r_antenna"]
DISK_MOTOR_ROLL_NAME : str = "neck_roll"
DISK_MOTOR_PITCH_NAME : str = "neck_pitch"
DISK_MOTOR_YAW_NAME : str = "neck_yaw"

DISK_MOTOR_NAME     : list = [DISK_MOTOR_ROLL_NAME, DISK_MOTOR_PITCH_NAME, DISK_MOTOR_YAW_NAME]


HEAD_MOTOR_NAME     : list = DISK_MOTOR_NAME + ANTENNA_MOTOR_NAME

# ─── Arm geometry (meters) ─────────────────────────────────────────────────────

ORIGIN_TO_SHOULDER : float = 0.19
SHOULDER_TO_ELBOW  : float = 0.28
ELBOW_TO_WRIST     : float = 0.25

# ─── Collision ─────────────────────────────────────────────────────────────────

CAPSULE_COLLISION_RADIUS : float = 0.04

TABLE_Z_COORD            : float = -0.4

# Torso capsule : origin [0,0,0] (shoulder height) to [0,0,-TORSO_SIZE].
# TORSO_COLLISION_RADIUS is kept small (0.03m) because the midpoint heuristic
# used for the forearm capsule is geometrically approximate — a conservative
# radius prevents false positives on the right arm whose FK segment crosses
# close to the torso axis in safe poses.
TORSO_SIZE               : float = 0.40
TORSO_COLLISION_RADIUS   : float = 0.03

SAFE_GOTO_STEPS : int = 5

# ─── Arm IDs ───────────────────────────────────────────────────────────────────

ARM_LEFT_ID  : str = "l"
ARM_RIGHT_ID : str = "r"
ARM_NAME     : str = "arm"