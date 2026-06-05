# ─── Motor names ───────────────────────────────────────────────────────────────

HAND_MOTOR_NAME     : str  = "gripper"
SHOULDER_MOTOR_NAME : list = ["shoulder_pitch", "shoulder_roll", "arm_yaw"]
ELBOW_MOTOR_NAME    : list = ["elbow_pitch", "forearm_yaw"]
FOREARM_MOTOR_NAME  : list = ["wrist_pitch", "wrist_roll", HAND_MOTOR_NAME]
ARM_MOTOR_NAME      : list = SHOULDER_MOTOR_NAME + ELBOW_MOTOR_NAME + FOREARM_MOTOR_NAME

ANTENNA_MOTOR_NAME    : list = ["l_antenna", "r_antenna"]
DISK_MOTOR_ROLL_NAME  : str  = "neck_roll"
DISK_MOTOR_PITCH_NAME : str  = "neck_pitch"
DISK_MOTOR_YAW_NAME   : str  = "neck_yaw"
TIME_SERIE_HEAD_VALUES_NAME : list = ["head_x", "head_y", "head_z"]
DISK_MOTOR_NAME : list = [DISK_MOTOR_ROLL_NAME, DISK_MOTOR_PITCH_NAME, DISK_MOTOR_YAW_NAME]
HEAD_MOTOR_NAME : list = DISK_MOTOR_NAME + ANTENNA_MOTOR_NAME

# ─── Arm geometry (meters) ─────────────────────────────────────────────────────

ORIGIN_TO_SHOULDER : float = 0.19
SHOULDER_TO_ELBOW  : float = 0.28
ELBOW_TO_WRIST     : float = 0.25

# ─── Collision capsule ─────────────────────────────────────────────────────────

CAPSULE_COLLISION_RADIUS : float = 0.04

# ─── Torso ─────────────────────────────────────────────────────────────────────

TORSO_SIZE             : float = 0.25
TORSO_COLLISION_RADIUS : float = 0.12

# ─── Table (AABB) ──────────────────────────────────────────────────────────────
#
# Table physique : 120 cm (longueur) × 80 cm (largeur) × 73 cm (hauteur)
#
# Repère Reachy : X+ = avant du robot, Y+ = gauche, Z+ = haut
#
# Placement :
#   - Reachy est centré sur une longueur de 120 cm  →  Y = axe de la longueur
#   - La table est DERRIÈRE Reachy                  →  X négatif
#   - L'arête côté robot est à x ≈ 0               →  X_MAX ≈ 0
#   - La table s'étend 80 cm derrière               →  X_MIN = X_MAX - 0.80
#
# Calibration terrain (bras gauche compliant, main posée sur les bords) :
#
#   Arête côté robot :
#     coin gauche : tip_x = -0.065
#     coin droit  : tip_x = -0.032
#     → moyenne = -0.049  →  X_MAX = 0.00  (arête arrondie à l'origine)
#
#   Bord arrière :
#     X_MIN = X_MAX - 0.80 = -0.80
#
#   Bord gauche  : tip_y = +0.503  →  Y_MAX = +0.50
#   Bord droit   : tip_y = -0.521  →  Y_MIN = -0.52
#
#   Surface Z :
#     tip sur la table : poignet z = -0.384 / -0.394
#     Le tip (point FK) est ~3 cm sous le poignet → surface ≈ -0.41
#     Z_MAX = -0.40  (valeur médiane, légèrement conservatrice)
#     À affiner : poser la main sur la table et vérifier que COLLISION
#     est bien détectée. Si non, descendre Z_MAX de 0.01 par pas.
#
#   Dessous plateau : épaisseur ~5 cm → Z_MIN = Z_MAX - 0.05

TABLE_X_MIN : float = -0.80   # bord arrière (80 cm derrière l'origine)
TABLE_X_MAX : float =  0.00   # arête côté robot
TABLE_Y_MIN : float = -0.60   # bord droit  (mesuré)
TABLE_Y_MAX : float =  0.60   # bord gauche (mesuré)
TABLE_Z_MAX : float = -0.40   # surface de la table (mesuré)
TABLE_Z_MIN : float = -1.18   # dessous du plateau (~5 cm)

# Conservé pour compatibilité ascendante
TABLE_Z_COORD : float = TABLE_Z_MAX

# ─── Motion ────────────────────────────────────────────────────────────────────

SAFE_GOTO_STEPS : int = 5

# ─── Arm IDs ───────────────────────────────────────────────────────────────────

ARM_LEFT_ID  : str = "l"
ARM_RIGHT_ID : str = "r"
ARM_NAME     : str = "arm"