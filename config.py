HAND_MOTOR_NAME     : str  = "gripper"
SHOULDER_MOTOR_NAME : list = ["shoulder_pitch", "shoulder_roll", "arm_yaw"]
ELBOW_MOTOR_NAME    : list = ["elbow_pitch", "forearm_yaw"]
FOREARM_MOTOR_NAME  : list = ["wrist_pitch", "wrist_roll", HAND_MOTOR_NAME]
ARM_MOTOR_NAME      : list = SHOULDER_MOTOR_NAME + ELBOW_MOTOR_NAME + FOREARM_MOTOR_NAME

ANTENNA_MOTOR_NAME    : str  = "antenna"
DISK_MOTOR_ROLL_NAME  : str  = "neck_roll"
DISK_MOTOR_PITCH_NAME : str  = "neck_pitch"
DISK_MOTOR_YAW_NAME   : str  = "neck_yaw"
TIME_SERIE_HEAD_VALUES_NAME : list = ["head_x", "head_y", "head_z"]
DISK_MOTOR_NAME : list = [DISK_MOTOR_ROLL_NAME, DISK_MOTOR_PITCH_NAME, DISK_MOTOR_YAW_NAME]
HEAD_MOTOR_NAME : list = DISK_MOTOR_NAME

ORIGIN_TO_SHOULDER : float = 0.19
SHOULDER_TO_ELBOW  : float = 0.28
ELBOW_TO_WRIST     : float = 0.25

CAPSULE_COLLISION_RADIUS : float = 0.04

TORSO_SIZE             : float = 0.25
TORSO_COLLISION_RADIUS : float = 0.12

# Table AABB bounds in Reachy frame (X+ = forward, Y+ = left, Z+ = up).
# Adjust these values to match your physical setup.
TABLE_X_MIN : float = -0.80
TABLE_X_MAX : float =  0.00
TABLE_Y_MIN : float = -0.60
TABLE_Y_MAX : float =  0.60
TABLE_Z_MAX : float = -0.40
TABLE_Z_MIN : float = -1.18

SAFE_GOTO_STEPS : int = 5

ARM_LEFT_ID  : str = "l"
ARM_RIGHT_ID : str = "r"
ARM_NAME     : str = "arm"

FAN_THRESHOLD : float = 35.0
