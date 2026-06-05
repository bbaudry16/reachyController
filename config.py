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
TIME_SERIE_HEAD_VALUES_NAME : list = ["head_x", "head_y", "head_z"]

DISK_MOTOR_NAME     : list = [DISK_MOTOR_ROLL_NAME, DISK_MOTOR_PITCH_NAME, DISK_MOTOR_YAW_NAME]

HEAD_MOTOR_NAME     : list = DISK_MOTOR_NAME + ANTENNA_MOTOR_NAME

# ─── Arm geometry (meters) ─────────────────────────────────────────────────────

ORIGIN_TO_SHOULDER : float = 0.19
SHOULDER_TO_ELBOW  : float = 0.28
ELBOW_TO_WRIST     : float = 0.25

# ─── Collision ─────────────────────────────────────────────────────────────────

CAPSULE_COLLISION_RADIUS : float = 0.02

# ── Table ──────────────────────────────────────────────────────────────────────
#
# La table est maintenant décrite comme une boîte AABB via TableCollider.
# Ces valeurs par défaut correspondent à un robot assis face à une table
# standard de 75 cm de hauteur, bord avant à 15 cm du torse.
#
# Repère Reachy :  X = avant,  Y = gauche,  Z = haut
# Origine         = centre torse, hauteur épaules (~115 cm du sol)
# Surface table   ≈ sol + 75 cm  →  115 - 75 = 40 cm sous les épaules → z = -0.40
#
# Ajuste ces valeurs selon la configuration physique réelle du robot.

TABLE_X_MIN : float =  0.10   # bord avant (côté robot) en m
TABLE_X_MAX : float =  0.80   # bord arrière de la table
TABLE_Y_MIN : float = -0.50   # bord gauche
TABLE_Y_MAX : float =  0.50   # bord droit
TABLE_Z_MIN : float = -0.50   # dessous de la table (châssis)
TABLE_Z_MAX : float = -0.40   # dessus / surface de la table

# Ancien TABLE_Z_COORD gardé pour compatibilité ascendante (non utilisé par armController)
TABLE_Z_COORD : float = TABLE_Z_MAX

# ── Torso ──────────────────────────────────────────────────────────────────────

TORSO_SIZE             : float = 0.40
TORSO_COLLISION_RADIUS : float = 0.12

SAFE_GOTO_STEPS : int = 5

# ─── Arm IDs ───────────────────────────────────────────────────────────────────

ARM_LEFT_ID  : str = "l"
ARM_RIGHT_ID : str = "r"
ARM_NAME     : str = "arm"