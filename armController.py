from __future__ import annotations
import numpy as np
from reachy_sdk import trajectory, ReachySDK
from scipy.spatial.transform import Rotation as R
import time

from . import config
from . import reachyPart as rp
from . import consoleManager as cm
from .timeSeries import TimeSeries
from .capsuleCollider import CapsuleCollider


# ─── DH-like kinematic chain (right arm, from the official docs) ───────────────
#
# Each entry: (translation_xyz, rotation_axis)
# The chain is built in the WORLD frame (origin = torso center, shoulder height).
# For the LEFT arm, the shoulder translation Y is flipped (+0.19 instead of -0.19),
# and the shoulder_roll axis convention is mirrored (handled by _sided sign below).
#
# translations are in meters, angles in degrees (converted internally).
#
# Joint order (matches config.ARM_MOTOR_NAME):
#   0  shoulder_pitch   trans=(0, ±0.19, 0)   axis=Y
#   1  shoulder_roll    trans=(0, 0, 0)        axis=X
#   2  arm_yaw          trans=(0, 0, 0)        axis=Z
#   3  elbow_pitch      trans=(0, 0, -0.28)    axis=Y
#   4  forearm_yaw      trans=(0, 0, 0)        axis=Z
#   5  wrist_pitch      trans=(0, 0, -0.25)    axis=Y
#   6  wrist_roll       trans=(0, 0, -0.0325)  axis=X
#   7  gripper          trans=(0, ∓0.01,-0.075) axis=None  (no rotation)
#
# The gripper translation Y is -0.01 for right, +0.01 for left (minor lateral offset).

_CHAIN_RIGHT = [
    # (translation, rotation_axis or None)
    (np.array([0.0,  -0.19,  0.0   ]), 'Y'),   # 0 shoulder_pitch
    (np.array([0.0,   0.0,   0.0   ]), 'X'),   # 1 shoulder_roll
    (np.array([0.0,   0.0,   0.0   ]), 'Z'),   # 2 arm_yaw
    (np.array([0.0,   0.0,  -0.28  ]), 'Y'),   # 3 elbow_pitch
    (np.array([0.0,   0.0,   0.0   ]), 'Z'),   # 4 forearm_yaw
    (np.array([0.0,   0.0,  -0.25  ]), 'Y'),   # 5 wrist_pitch
    (np.array([0.0,   0.0,  -0.0325]), 'X'),   # 6 wrist_roll
    (np.array([0.0,  -0.01, -0.075 ]), None),  # 7 gripper (end-effector, no rot)
]

_CHAIN_LEFT = [
    (np.array([0.0,   0.19,  0.0   ]), 'Y'),   # 0 shoulder_pitch
    (np.array([0.0,   0.0,   0.0   ]), 'X'),   # 1 shoulder_roll
    (np.array([0.0,   0.0,   0.0   ]), 'Z'),   # 2 arm_yaw
    (np.array([0.0,   0.0,  -0.28  ]), 'Y'),   # 3 elbow_pitch
    (np.array([0.0,   0.0,   0.0   ]), 'Z'),   # 4 forearm_yaw
    (np.array([0.0,   0.0,  -0.25  ]), 'Y'),   # 5 wrist_pitch
    (np.array([0.0,   0.0,  -0.0325]), 'X'),   # 6 wrist_roll
    (np.array([0.0,   0.01, -0.075 ]), None),  # 7 gripper
]


def _rot_matrix(axis: str, angle_deg: float) -> np.ndarray:
    """Return a 3x3 rotation matrix for a rotation around X, Y or Z."""
    return R.from_euler(axis, angle_deg, degrees=True).as_matrix()


def analytical_fk(chain: list, joint_angles_deg: list) -> list[np.ndarray]:
    """
    Compute the world-space position of every joint frame origin along the chain.

    Parameters
    ----------
    chain : list of (translation_vec3, axis_or_None)
    joint_angles_deg : list of floats, one per entry that has an axis.
                       Length must equal the number of joints WITH a rotation axis.

    Returns
    -------
    positions : list of np.ndarray shape (3,)
        positions[0]  = world origin (before any transform)
        positions[1]  = after shoulder_pitch frame  (= shoulder position)
        positions[2]  = after shoulder_roll
        positions[3]  = after arm_yaw
        positions[4]  = after elbow translation  (= elbow position)
        positions[5]  = after forearm_yaw
        positions[6]  = after wrist translation  (= wrist position)
        positions[7]  = after wrist_roll
        positions[8]  = end-effector (gripper tip)
    """
    pos   = np.zeros(3)
    rot   = np.eye(3)
    angle_idx = 0

    positions = [pos.copy()]   # world origin

    for (trans, axis) in chain:
        # 1. translate in current frame
        pos = pos + rot @ trans

        # 2. rotate if this joint has a DOF
        if axis is not None:
            angle = joint_angles_deg[angle_idx]
            angle_idx += 1
            rot = rot @ _rot_matrix(axis, angle)

        positions.append(pos.copy())

    return positions


class ReachyJoint:
    """
    Describe reachy joint with a max and a min angle.
    PARAMETER maxAngleEuler : float, minAngleEuler : float
    """
    def __init__(self, maxAngleEuler: float, minAngleEuler: float):
        self.maxAngle = maxAngleEuler
        self.minAngle = minAngleEuler


class ReachyArm(rp.ReachyPart):

    # ─── Joint constraints ─────────────────────────────────────────────────────
    JOINT_SHOULDER_PITCH = ReachyJoint(90.0,  -150.0)
    JOINT_SHOULDER_ROLL  = ReachyJoint(10.0,  -180.0)
    JOINT_ARM_YAW        = ReachyJoint(90.0,   -90.0)
    JOINT_ELBOW_PITCH    = ReachyJoint(0.0,   -125.0)
    JOINT_FOREARM_YAW    = ReachyJoint(100.0, -100.0)
    JOINT_WRIST_PITCH    = ReachyJoint(45.0,   -45.0)
    JOINT_WRIST_ROLL     = ReachyJoint(35,       -55)
    JOINT_GRIPPER        = ReachyJoint(20.0,   -69.0)

    # ─── Console ───────────────────────────────────────────────────────────────
    CLASS_NAME  : str = "Reachy arm"
    CLASS_COLOR : str = cm.Color.CYAN

    def __init__(self, _reachy: ReachySDK, _armID: str, collisionManager=None) -> None:
        self._armID  : str = _armID
        self._setupConstraints()

        self._reachyArm         = getattr(_reachy, self._sided(config.ARM_NAME))
        self._joints : dict     = self._setupJoints()
        self._joint_constraints = self._setupJointConstraints()

        # Select the kinematic chain for this arm
        self._chain = _CHAIN_LEFT if _armID == config.ARM_LEFT_ID else _CHAIN_RIGHT

        self._collisionManager  = collisionManager
        if self._collisionManager is None:
            cm.MKprintSafety("No collision manager set, inter-part collision will be ignored.", self.CLASS_NAME, self.CLASS_COLOR)

        self.canMove                 : bool = True
        self.hasNotifyImpossibleMove : bool = False
        self.collisionWithTableOn    : bool = False

        # Cible prédite du step courant dans _safeGoto.
        # Publiée AVANT le check pour que le CollisionManager puisse la lire
        # depuis l'autre bras lors d'un mouvement parallel.
        self._pendingJointDict : dict | None = None

    # ─── Setup ─────────────────────────────────────────────────────────────────

    def activateCollisionWithTable(self):
        self.collisionWithTableOn = True

    def desactivateCollisionWithTable(self):
        self.collisionWithTableOn = False

    def _setupConstraints(self) -> None:
        if self._armID == config.ARM_LEFT_ID:
            self.JOINT_SHOULDER_ROLL = ReachyJoint(180.0, -10.0)
            self.JOINT_WRIST_ROLL    = ReachyJoint(55, -35)
            self.JOINT_GRIPPER       = ReachyJoint(69, -20)

    def _setupJoints(self) -> dict:
        r = {}
        for name in config.ARM_MOTOR_NAME:
            sided = self._sided(name)
            r[sided] = getattr(self._reachyArm, sided)
        return r

    def getJointsInOrder(self) -> list:
        return [self._joints[self._sided(name)] for name in config.ARM_MOTOR_NAME]

    def _setupJointConstraints(self) -> dict:
        return {
            self._sided("shoulder_pitch"): self.JOINT_SHOULDER_PITCH,
            self._sided("shoulder_roll"):  self.JOINT_SHOULDER_ROLL,
            self._sided("arm_yaw"):        self.JOINT_ARM_YAW,
            self._sided("elbow_pitch"):    self.JOINT_ELBOW_PITCH,
            self._sided("forearm_yaw"):    self.JOINT_FOREARM_YAW,
            self._sided("wrist_pitch"):    self.JOINT_WRIST_PITCH,
            self._sided("wrist_roll"):     self.JOINT_WRIST_ROLL,
            self._sided("gripper"):        self.JOINT_GRIPPER,
        }

    # ─── Public setters ────────────────────────────────────────────────────────

    def setCollisionManager(self, collisionManager) -> None:
        self._collisionManager = collisionManager
        cm.MKprintSafety("Collision manager set — inter-part collision is now active.", self.CLASS_NAME, self.CLASS_COLOR)

    def getArmId(self) -> str:
        return self._armID

    def resetCanMove(self) -> None:
        self.canMove = True
        self.hasNotifyImpossibleMove = False

    # ─── Internal helpers ──────────────────────────────────────────────────────

    def _sided(self, name: str) -> str:
        return self._armID + "_" + name

    def _clamp(self, jointName: str, value: float, min_v: float, max_v: float) -> float:
        r = max(min(value, max_v), min_v)
        if value < min_v or value > max_v:
            cm.MKprintSafety(f"{jointName} clamped to {r}", self.CLASS_NAME, self.CLASS_COLOR)
        return r

    # ─── Analytical FK ─────────────────────────────────────────────────────────

    def _getCurrentAngles(self) -> list:
        """Return the 8 joint angles (deg) in chain order from live joints."""
        ordered = ["shoulder_pitch", "shoulder_roll", "arm_yaw",
                   "elbow_pitch", "forearm_yaw", "wrist_pitch", "wrist_roll", "gripper"]
        return [self._joints[self._sided(j)].present_position for j in ordered]

    def _anglesFromJointDict(self, joint_dict: dict) -> list:
        """Return the 8 joint angles (deg) in chain order from a joint_dict.
        joint_dict keys are SDK joint objects.
        If a joint is absent (e.g. gripper not passed by move_joints),
        falls back to its current present_position."""
        ordered = ["shoulder_pitch", "shoulder_roll", "arm_yaw",
                   "elbow_pitch", "forearm_yaw", "wrist_pitch", "wrist_roll", "gripper"]
        result = []
        for j in ordered:
            sdk_joint = self._joints[self._sided(j)]
            result.append(joint_dict.get(sdk_joint, sdk_joint.present_position))
        return result

    def computeFK(self, joint_angles_deg: list | None = None) -> list[np.ndarray]:
        """
        Compute the full forward kinematics for this arm.

        Parameters
        ----------
        joint_angles_deg : list of 8 floats (shoulder_pitch … gripper), degrees.
                           If None, uses current live joint positions.

        Returns
        -------
        positions : list of 9 np.ndarray (world-space XYZ)
            [0] world origin
            [1] shoulder  (after shoulder_pitch translation)
            [2] after shoulder_roll
            [3] after arm_yaw
            [4] elbow     (after elbow_pitch translation)
            [5] after forearm_yaw
            [6] wrist     (after wrist_pitch translation)
            [7] after wrist_roll
            [8] end-effector / gripper tip
        """
        if joint_angles_deg is None:
            joint_angles_deg = self._getCurrentAngles()
        return analytical_fk(self._chain, joint_angles_deg)

    def computeFKFromJointDict(self, joint_dict: dict) -> list[np.ndarray]:
        """Compute FK from a joint_dict (keys = SDK joint objects)."""
        return self.computeFK(self._anglesFromJointDict(joint_dict))

    # ─── Collision checks ──────────────────────────────────────────────────────

    def _collideWithTable(self, joint_dict: dict) -> bool:
        positions = self.computeFKFromJointDict(joint_dict)
        # Check wrist and end-effector against table plane
        for pt in positions[6:]:   # wrist, wrist_roll, gripper
            if pt[2] <= config.TABLE_Z_COORD:
                return True
        return False

    def _collideWithOtherPart(self, joint_dict: dict) -> bool:
        if self._collisionManager is None:
            return False

        # Récupère la cible prédite de l'autre bras s'il bouge en parallel.
        # Si l'autre bras n'a pas de _pendingJointDict (pas en mouvement),
        # askValidMovementBoth utilisera sa position live à la place (None).
        other_arm = (self._collisionManager._armLeft
                     if self._armID == self._collisionManager._armRight.getArmId()
                     else self._collisionManager._armRight)
        other_pending = getattr(other_arm, "_pendingJointDict", None)

        if self._armID == self._collisionManager._armRight.getArmId():
            right_ok, _ = self._collisionManager.askValidMovementBoth(joint_dict, other_pending)
            return not right_ok
        else:
            _, left_ok = self._collisionManager.askValidMovementBoth(other_pending, joint_dict)
            return not left_ok

    def _checkCollision(self, joint_dict: dict) -> bool:
        if self.collisionWithTableOn and self._collideWithTable(joint_dict):
            cm.MKprintSafety("Collision with table!", self.CLASS_NAME, self.CLASS_COLOR)
            self.canMove = False
            return True
        if self._collideWithOtherPart(joint_dict):
            cm.MKprintSafety("Collision with another part!", self.CLASS_NAME, self.CLASS_COLOR)
            self.canMove = False
            return True
        return False

    # ─── Collision shapes ──────────────────────────────────────────────────────
    #
    # The arm is now modelled with 3 accurate capsules using real joint positions
    # from the analytical FK:
    #
    #   capsule 0 : shoulder → elbow        (upper arm)
    #   capsule 1 : elbow    → wrist        (forearm)
    #   capsule 2 : wrist    → end-effector (hand/gripper)
    #
    # This replaces the previous midpoint heuristic which produced incorrect
    # segment lengths and caused both false positives (torso too close on right
    # arm) and false negatives (gaps between actual arm geometry and capsules).
    #
    # For torso collision, capsule 0 (upper arm) is still excluded to avoid
    # false positives when the arm hangs at the side — the shoulder-to-elbow
    # segment passes near the torso in safe poses. Capsules 1 and 2 are checked.

    def getCollision(self) -> list[CapsuleCollider]:
        """Return collision capsules for the current arm pose (live joints)."""
        positions = self.computeFK()
        return self._capsulesFromFK(positions)

    def getCollisionFromPosition(self, joint_dict: dict) -> list[CapsuleCollider]:
        """Return collision capsules for a given joint dict (predictive check)."""
        positions = self.computeFKFromJointDict(joint_dict)
        return self._capsulesFromFK(positions)

    def _capsulesFromFK(self, positions: list[np.ndarray]) -> list[CapsuleCollider]:
        """
        Build the 3 arm capsules from a list of FK positions.

        positions[1] = shoulder
        positions[4] = elbow
        positions[6] = wrist
        positions[8] = end-effector
        """
        shoulder = positions[1].tolist()
        elbow    = positions[4].tolist()
        wrist    = positions[6].tolist()
        tip      = positions[8].tolist()
        r        = config.CAPSULE_COLLISION_RADIUS
        return [
            CapsuleCollider(shoulder, elbow, r),   # upper arm
            CapsuleCollider(elbow,    wrist, r),   # forearm
            CapsuleCollider(wrist,    tip,   r),   # hand
        ]

    # ─── Motion ────────────────────────────────────────────────────────────────

    def getInterpoaltionByName(self, name: str) -> "trajectory.interpolation":
        return getattr(trajectory.interpolation, name)

    def safeGoto(self, joint_dict: dict, duration: float, interpolation=trajectory.interpolation.linear, steps: int = config.SAFE_GOTO_STEPS) -> None:
        cm.MKprint("going safely in " + str(duration) + "s at pose : " + str(joint_dict) + ", using " + str(steps) + " collision check", self.CLASS_NAME, self.CLASS_COLOR)
        self._safeGoto(joint_dict, duration, interpolation, steps)

    def _safeGoto(self, joint_dict: dict, duration: float, interpolation=trajectory.interpolation.linear, steps: int = config.SAFE_GOTO_STEPS) -> None:
        if not self.canMove:
            if not self.hasNotifyImpossibleMove:
                cm.MKprintSafety("Cannot safely move — please reset Reachy position.", self.CLASS_NAME, self.CLASS_COLOR)
                self.hasNotifyImpossibleMove = True
            return

        safe_target = {}
        for joint, pos in joint_dict.items():
            name = joint.name
            if name in self._joint_constraints:
                limits = self._joint_constraints[name]
                pos = self._clamp(name, pos, limits.minAngle, limits.maxAngle)
            safe_target[joint] = pos

        start_positions = {joint: joint.present_position for joint in safe_target}
        step_duration = duration / steps

        for i in range(1, steps + 1):
            alpha = i / steps
            interpolated = {
                joint: start_positions[joint] + alpha * (safe_target[joint] - start_positions[joint])
                for joint in safe_target
            }
            # Publier la cible prédite pour que l'autre bras puisse la lire
            # lors d'un parallel (avant le check, pas après).
            self._pendingJointDict = interpolated
            if self._checkCollision(interpolated):
                self._pendingJointDict = None
                return
            trajectory.goto(interpolated, duration=step_duration, interpolation_mode=interpolation)

        self._pendingJointDict = None

    def _debug_goto(self, joint_dict: dict, duration: float, interpolation=trajectory.interpolation.linear) -> None:
        """Not safe — debug use only."""
        trajectory.goto(joint_dict, duration=duration, interpolation_mode=interpolation)

    def _debug_placeHandOnTable(self, duration: float = 1.0) -> None:
        target = {
            self._joints[self._sided("shoulder_pitch")]: 0.0,
            self._joints[self._sided("shoulder_roll")]:  0.0,
            self._joints[self._sided("arm_yaw")]:        0.0,
            self._joints[self._sided("elbow_pitch")]:   -90.0,
            self._joints[self._sided("forearm_yaw")]:    0.0,
            self._joints[self._sided("wrist_pitch")]:    0.0,
            self._joints[self._sided("wrist_roll")]:     0.0,
        }
        cm.MKprintDebug("Resetting arm position, ignoring obstacles.", self.CLASS_NAME, self.CLASS_COLOR)
        self._debug_goto(target, duration=duration)

    def gotoCartesianPoint(self, goalPosition: list, goalRotation: list, duration: float = 0.1, interpolation=trajectory.interpolation.linear) -> None:
        IKMatrix = self._getIKMatrix(goalPosition, goalRotation)
        jointPos = self._reachyArm.inverse_kinematics(IKMatrix)
        if self.canMove:
            cm.MKprint(f"Going to {goalPosition} with rotation {goalRotation} in {duration}s.", self.CLASS_NAME, self.CLASS_COLOR)
        self._safeGoto(
            {joint: pos for joint, pos in zip(self._reachyArm.joints.values(), jointPos)},
            duration=duration,
            interpolation=interpolation
        )

    def changeHandAngle(self, angleEuler: float, duration: float) -> None:
        gripperName  = self._sided(config.HAND_MOTOR_NAME)
        gripperJoint = self._joints[gripperName]
        safe_angle   = self._clamp(gripperName, angleEuler, self.JOINT_GRIPPER.minAngle, self.JOINT_GRIPPER.maxAngle)
        self._safeGoto({gripperJoint: safe_angle}, duration=duration)

    def openHand(self, duration: float = 0.5) -> None:
        cm.MKprint("opening hand in " + str(duration) + "s", self.CLASS_NAME, self.CLASS_COLOR)
        if self._armID == config.ARM_RIGHT_ID:
            self.changeHandAngle(self.JOINT_GRIPPER.minAngle, duration)
        else:
            self.changeHandAngle(self.JOINT_GRIPPER.maxAngle, duration)

    def closeHand(self, duration: float = 0.5) -> None:
        cm.MKprint("closing hand in " + str(duration) + "s", self.CLASS_NAME, self.CLASS_COLOR)
        if self._armID == config.ARM_RIGHT_ID:
            self.changeHandAngle(self.JOINT_GRIPPER.maxAngle, duration)
        else:
            self.changeHandAngle(self.JOINT_GRIPPER.minAngle, duration)

    # ─── Kinematics (SDK-based, kept for IK / cartesian goto) ──────────────────

    def _eulerToMatrix(self, anglesDeg: list):
        return R.from_euler("xyz", anglesDeg, True).as_matrix()

    def _getIKMatrix(self, goalPosition: list, goalRotationDeg: list):
        rm = self._eulerToMatrix(goalRotationDeg)
        return np.array([
            [rm[0][0], rm[0][1], rm[0][2], goalPosition[0]],
            [rm[1][0], rm[1][1], rm[1][2], goalPosition[1]],
            [rm[2][0], rm[2][1], rm[2][2], goalPosition[2]],
            [0,        0,        0,        1              ],
        ])

    def getShoulderPosition(self) -> list:
        """Shoulder position in world frame from analytical FK."""
        positions = self.computeFK()
        return positions[1].tolist()

    def getHandPositionFromForwardKinematicsMatrix(self, fk) -> list:
        return [float(fk[0][3]), float(fk[1][3]), float(fk[2][3])]

    def getHandPosition(self) -> list:
        """Hand (end-effector) position from analytical FK."""
        positions = self.computeFK()
        return positions[8].tolist()

    def getHandPositionFromJointsPosition(self, joint_dict: dict) -> list:
        """Hand position from a joint_dict via analytical FK."""
        positions = self.computeFKFromJointDict(joint_dict)
        return positions[8].tolist()

    # ─── Record / Play ─────────────────────────────────────────────────────────

    def recordArm(self, recordDurationSeconds: float, samplingFrequencyHertz: float) -> TimeSeries:
        trajectories = []
        samplingTime = 1.0 / samplingFrequencyHertz
        start        = time.time()
        last_valid   = {}

        cm.MKprint(f"Recording {self._sided(config.ARM_NAME)} for {recordDurationSeconds}s at {samplingFrequencyHertz}Hz.", self.CLASS_NAME, self.CLASS_COLOR)

        while (time.time() - start) < recordDurationSeconds:
            frame = {name: joint.present_position for name, joint in self._joints.items()}
            if last_valid and all(v == 0.0 for v in frame.values()):
                cm.MKprintSafety("Suspicious all-zero frame detected during recording — using last valid frame.", self.CLASS_NAME, self.CLASS_COLOR)
                frame = last_valid.copy()
            else:
                last_valid = frame
            trajectories.append(frame)
            time.sleep(samplingTime)

        cm.MKprint(f"Recording done for {self._sided(config.ARM_NAME)}.", self.CLASS_NAME, self.CLASS_COLOR)
        return TimeSeries(samplingFrequencyHertz, recordDurationSeconds, trajectories, [self._armID == config.ARM_RIGHT_ID, self._armID == config.ARM_LEFT_ID, 0])

    def playArmRecord(self, record: TimeSeries, startDuration: float = 3.0, collisionCheckInterval: int = 5) -> None:
        if not self.canMove:
            if not self.hasNotifyImpossibleMove:
                cm.MKprintSafety("Cannot safely move — please reset Reachy position.", self.CLASS_NAME, self.CLASS_COLOR)
                self.hasNotifyImpossibleMove = True
            return

        firstPoint = {
            self._joints[m]: pos
            for m, pos in record.jointPosition[0].items()
            if m in self._joints
        }
        samplingTime = 1.0 / record.samplingFrequency

        cm.MKprintDebug(f"samplingTime={samplingTime} samplingFrequency={record.samplingFrequency}", self.CLASS_NAME, self.CLASS_COLOR)
        cm.MKprint(f"Playing record for {self._sided(config.ARM_NAME)}.", self.CLASS_NAME, self.CLASS_COLOR)

        self._safeGoto(firstPoint, duration=startDuration)

        for i, jointsPositions in enumerate(record.jointPosition):
            t_start = time.time()

            safe_step = {
                self._joints[name]: pos
                for name, pos in jointsPositions.items()
                if name in self._joints
            }

            if i > 0 and collisionCheckInterval > 0 and i % collisionCheckInterval == 0:
                if self._checkCollision(safe_step):
                    return

            for name, pos in jointsPositions.items():
                if name in self._joints:
                    self._joints[name].goal_position = pos

            elapsed   = time.time() - t_start
            remaining = samplingTime - elapsed
            if remaining > 0:
                time.sleep(remaining)