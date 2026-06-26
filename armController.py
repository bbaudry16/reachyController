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
from .tableCollider import TableCollider


# DH-like kinematic chain definition.
# Each entry: (translation_xyz, rotation_axis or None).
# World frame origin is at torso center at shoulder height.
#
# Joint order matches config.ARM_MOTOR_NAME:
#   0  shoulder_pitch   trans=(0, -0.19, 0)    axis=Y   (right) / +0.19 (left)
#   1  shoulder_roll    trans=(0,  0,    0)    axis=X
#   2  arm_yaw          trans=(0,  0,    0)    axis=Z
#   3  elbow_pitch      trans=(0,  0,   -0.28) axis=Y
#   4  forearm_yaw      trans=(0,  0,    0)    axis=Z
#   5  wrist_pitch      trans=(0,  0,   -0.25) axis=Y
#   6  wrist_roll       trans=(0,  0,   -0.0325) axis=X
#   7  gripper          trans=(0, -0.01,-0.075)  axis=None (end-effector)

_CHAIN_RIGHT = [
    (np.array([0.0,  -0.19,  0.0    ]), 'Y'),
    (np.array([0.0,   0.0,   0.0    ]), 'X'),
    (np.array([0.0,   0.0,   0.0    ]), 'Z'),
    (np.array([0.0,   0.0,  -0.28   ]), 'Y'),
    (np.array([0.0,   0.0,   0.0    ]), 'Z'),
    (np.array([0.0,   0.0,  -0.25   ]), 'Y'),
    (np.array([0.0,   0.0,  -0.0325 ]), 'X'),
    (np.array([0.0,  -0.01, -0.075  ]), None),
]

_CHAIN_LEFT = [
    (np.array([0.0,   0.19,  0.0    ]), 'Y'),
    (np.array([0.0,   0.0,   0.0    ]), 'X'),
    (np.array([0.0,   0.0,   0.0    ]), 'Z'),
    (np.array([0.0,   0.0,  -0.28   ]), 'Y'),
    (np.array([0.0,   0.0,   0.0    ]), 'Z'),
    (np.array([0.0,   0.0,  -0.25   ]), 'Y'),
    (np.array([0.0,   0.0,  -0.0325 ]), 'X'),
    (np.array([0.0,   0.01, -0.075  ]), None),
]


def _rot_matrix(axis: str, angle_deg: float) -> np.ndarray:
    """
    Return a 3x3 rotation matrix for a rotation around axis X, Y, or Z.

    @param axis: Rotation axis ('X', 'Y', or 'Z').
    @type axis: str
    @param angle_deg: Rotation angle in degrees.
    @type angle_deg: float
    @rtype: numpy.ndarray
    """
    return R.from_euler(axis, angle_deg, degrees=True).as_matrix()


def analytical_fk(chain: list, joint_angles_deg: list) -> list[np.ndarray]:
    """
    Compute the world-space position of every joint frame origin along the chain.

    @param chain: List of (translation_vec3, axis_or_None) tuples.
    @type chain: list
    @param joint_angles_deg: One angle per joint that has a rotation axis.
    @type joint_angles_deg: list[float]
    @rtype: list[numpy.ndarray]
    @return: 9 world-space XYZ positions:
        [0] world origin,
        [1] shoulder,
        [2] after shoulder_roll,
        [3] after arm_yaw,
        [4] elbow,
        [5] after forearm_yaw,
        [6] wrist,
        [7] after wrist_roll,
        [8] end-effector (gripper tip).
    """
    pos       = np.zeros(3)
    rot       = np.eye(3)
    angle_idx = 0
    positions = [pos.copy()]

    for (trans, axis) in chain:
        pos = pos + rot @ trans
        if axis is not None:
            rot = rot @ _rot_matrix(axis, joint_angles_deg[angle_idx])
            angle_idx += 1
        positions.append(pos.copy())

    return positions


class ReachyJoint:
    """
    Angular limits for a single Reachy joint.

    @ivar maxAngle: Maximum allowed angle in degrees.
    @ivar minAngle: Minimum allowed angle in degrees.
    """

    def __init__(self, maxAngleEuler: float, minAngleEuler: float):
        """
        @param maxAngleEuler: Maximum angle in degrees.
        @type maxAngleEuler: float
        @param minAngleEuler: Minimum angle in degrees.
        @type minAngleEuler: float
        """
        self.maxAngle = maxAngleEuler
        self.minAngle = minAngleEuler


class ReachyArm(rp.ReachyPart):
    """
    Controller for a single Reachy arm with collision checking and motion safety.

    Provides safe joint-space and Cartesian motion, analytical forward kinematics,
    capsule collision geometry, and record/replay support.

    @cvar JOINT_SHOULDER_PITCH: Shoulder pitch limits.
    @cvar JOINT_SHOULDER_ROLL: Shoulder roll limits (mirrored for left arm).
    @cvar JOINT_ARM_YAW: Arm yaw limits.
    @cvar JOINT_ELBOW_PITCH: Elbow pitch limits.
    @cvar JOINT_FOREARM_YAW: Forearm yaw limits.
    @cvar JOINT_WRIST_PITCH: Wrist pitch limits.
    @cvar JOINT_WRIST_ROLL: Wrist roll limits (mirrored for left arm).
    @cvar JOINT_GRIPPER: Gripper limits (mirrored for left arm).
    @cvar CLASS_NAME: Display name used in console output.
    @cvar CLASS_COLOR: Console color used for this class.
    """

    JOINT_SHOULDER_PITCH = ReachyJoint(90.0,  -150.0)
    JOINT_SHOULDER_ROLL  = ReachyJoint(10.0,  -180.0)
    JOINT_ARM_YAW        = ReachyJoint(90.0,   -90.0)
    JOINT_ELBOW_PITCH    = ReachyJoint(0.0,   -125.0)
    JOINT_FOREARM_YAW    = ReachyJoint(100.0, -100.0)
    JOINT_WRIST_PITCH    = ReachyJoint(45.0,   -45.0)
    JOINT_WRIST_ROLL     = ReachyJoint(35,       -55)
    JOINT_GRIPPER        = ReachyJoint(20.0,   -69.0)

    CLASS_NAME  : str = "Reachy arm"
    CLASS_COLOR : str = cm.Color.CYAN

    def __init__(self, _reachy: ReachySDK, _armID: str, collisionManager=None) -> None:
        """
        @param _reachy: Connected Reachy SDK instance.
        @type _reachy: ReachySDK
        @param _armID: Arm side identifier, 'l' or 'r'.
        @type _armID: str
        @param collisionManager: Optional collision manager. If None, inter-part
            collision is disabled.
        """
        self._armID = _armID
        self._setupConstraints()

        self._reachyArm         = getattr(_reachy, self._sided(config.ARM_NAME))
        self._joints : dict     = self._setupJoints()
        self._joint_constraints = self._setupJointConstraints()
        self._chain             = _CHAIN_LEFT if _armID == config.ARM_LEFT_ID else _CHAIN_RIGHT
        self._collisionManager  = collisionManager

        if self._collisionManager is None:
            cm.MKprintSafety(
                "No collision manager set, inter-part collision will be ignored.",
                self.CLASS_NAME, self.CLASS_COLOR
            )

        self.canMove                 : bool          = True
        self.hasNotifyImpossibleMove : bool          = False
        self.collisionWithTableOn    : bool          = False
        self._pendingJointDict       : dict | None   = None
        self._tableCollider          : TableCollider | None = None

    def activateCollisionWithTable(self, table: "TableCollider | None" = None) -> None:
        """
        Enable table collision checking.

        If no table is provided and none is set, one is created from config defaults.

        @param table: Optional table collider to use. Replaces any existing one.
        @type table: TableCollider or None
        """
        self.collisionWithTableOn = True
        if table is not None:
            self._tableCollider = table
        elif self._tableCollider is None:
            self._tableCollider = TableCollider(
                config.TABLE_X_MIN, config.TABLE_X_MAX,
                config.TABLE_Y_MIN, config.TABLE_Y_MAX,
                config.TABLE_Z_MIN, config.TABLE_Z_MAX,
            )

    def desactivateCollisionWithTable(self) -> None:
        """Disable table collision checking."""
        self.collisionWithTableOn = False

    def setTable(self, table: "TableCollider") -> None:
        """
        Replace the current table collider without changing the active/inactive state.

        @param table: New table collider.
        @type table: TableCollider
        """
        self._tableCollider = table

    def getTable(self) -> "TableCollider | None":
        """
        Return the current table collider, or None if none is set.

        @rtype: TableCollider or None
        """
        return self._tableCollider

    def _setupConstraints(self) -> None:
        """Mirror joint limits for the left arm."""
        if self._armID == config.ARM_LEFT_ID:
            self.JOINT_SHOULDER_ROLL = ReachyJoint(180.0, -10.0)
            self.JOINT_WRIST_ROLL    = ReachyJoint(55, -35)
            self.JOINT_GRIPPER       = ReachyJoint(69, -20)

    def _setupJoints(self) -> dict:
        """
        Build the joint dictionary from the SDK arm object.

        @rtype: dict
        """
        return {
            self._sided(name): getattr(self._reachyArm, self._sided(name))
            for name in config.ARM_MOTOR_NAME
        }

    def getJointsInOrder(self) -> list:
        """
        Return joint objects in the canonical motor order.

        @rtype: list
        """
        return [self._joints[self._sided(name)] for name in config.ARM_MOTOR_NAME]

    def _setupJointConstraints(self) -> dict:
        """
        Build the joint constraint dictionary keyed by sided motor name.

        @rtype: dict
        """
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

    def setCollisionManager(self, collisionManager) -> None:
        """
        Assign a collision manager, enabling inter-part collision checks.

        @param collisionManager: CollisionManager instance.
        """
        self._collisionManager = collisionManager
        cm.MKprintSafety(
            "Collision manager set — inter-part collision is now active.",
            self.CLASS_NAME, self.CLASS_COLOR
        )

    def getArmId(self) -> str:
        """
        Return the arm side identifier.

        @rtype: str
        """
        return self._armID

    def resetCanMove(self) -> None:
        """Reset the movement lock after a collision has been resolved."""
        self.canMove = True
        self.hasNotifyImpossibleMove = False

    def _sided(self, name: str) -> str:
        """
        Prefix a motor name with the arm side identifier.

        @param name: Base motor name.
        @type name: str
        @rtype: str
        """
        return self._armID + "_" + name

    def _clamp(self, jointName: str, value: float, min_v: float, max_v: float) -> float:
        """
        Clamp a joint angle to its allowed range and warn if clamping occurs.

        @param jointName: Motor name for logging.
        @type jointName: str
        @param value: Requested angle in degrees.
        @type value: float
        @param min_v: Minimum allowed angle.
        @type min_v: float
        @param max_v: Maximum allowed angle.
        @type max_v: float
        @rtype: float
        """
        r = max(min(value, max_v), min_v)
        if value < min_v or value > max_v:
            cm.MKprintSafety(f"{jointName} clamped to {r}", self.CLASS_NAME, self.CLASS_COLOR)
        return r

    def _getCurrentAngles(self) -> list:
        """
        Return the 8 joint angles in chain order from live joint readings.

        @rtype: list[float]
        """
        ordered = ["shoulder_pitch", "shoulder_roll", "arm_yaw",
                   "elbow_pitch", "forearm_yaw", "wrist_pitch", "wrist_roll", "gripper"]
        return [self._joints[self._sided(j)].present_position for j in ordered]

    def _anglesFromJointDict(self, joint_dict: dict) -> list:
        """
        Return the 8 joint angles in chain order from a joint_dict.

        Falls back to the present position for any joint not in the dict.

        @param joint_dict: Mapping of SDK joint object to target angle.
        @type joint_dict: dict
        @rtype: list[float]
        """
        ordered = ["shoulder_pitch", "shoulder_roll", "arm_yaw",
                   "elbow_pitch", "forearm_yaw", "wrist_pitch", "wrist_roll", "gripper"]
        result = []
        for j in ordered:
            sdk_joint = self._joints[self._sided(j)]
            result.append(joint_dict.get(sdk_joint, sdk_joint.present_position))
        return result

    def computeFK(self, joint_angles_deg: list | None = None) -> list[np.ndarray]:
        """
        Compute forward kinematics for this arm.

        @param joint_angles_deg: 8 joint angles in degrees (shoulder_pitch to gripper).
            If None, uses current live joint positions.
        @type joint_angles_deg: list[float] or None
        @rtype: list[numpy.ndarray]
        @return: 9 world-space XYZ positions (see L{analytical_fk} for index mapping).
        """
        if joint_angles_deg is None:
            joint_angles_deg = self._getCurrentAngles()
        return analytical_fk(self._chain, joint_angles_deg)

    def computeFKFromJointDict(self, joint_dict: dict) -> list[np.ndarray]:
        """
        Compute forward kinematics from a joint_dict.

        @param joint_dict: Mapping of SDK joint object to target angle.
        @type joint_dict: dict
        @rtype: list[numpy.ndarray]
        """
        return self.computeFK(self._anglesFromJointDict(joint_dict))

    def _collideWithTable(self, joint_dict: dict) -> bool:
        """
        Return True if any key arm point enters the table AABB.

        Checks positions from elbow onward (indices 4-8).

        @param joint_dict: Mapping of SDK joint object to target angle.
        @type joint_dict: dict
        @rtype: bool
        """
        if self._tableCollider is None:
            return False
        positions = self.computeFKFromJointDict(joint_dict)
        for pt in positions[4:]:
            if self._tableCollider.containsPoint(pt):
                return True
        return False

    def _collideWithOtherPart(self, joint_dict: dict) -> bool:
        """
        Return True if this arm collides with the other arm or torso.

        Uses the other arm's pending joint dict when available (parallel move),
        otherwise falls back to its live position.

        @param joint_dict: Target joint positions for this arm.
        @type joint_dict: dict
        @rtype: bool
        """
        if self._collisionManager is None:
            return False

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
        """
        Run all collision checks and lock movement on first detected collision.

        @param joint_dict: Target joint positions to check.
        @type joint_dict: dict
        @rtype: bool
        @return: True if a collision was detected.
        """
        if self.collisionWithTableOn and self._collideWithTable(joint_dict):
            cm.MKprintSafety("Collision with table!", self.CLASS_NAME, self.CLASS_COLOR)
            self.canMove = False
            return True
        if self._collideWithOtherPart(joint_dict):
            cm.MKprintSafety("Collision with another part!", self.CLASS_NAME, self.CLASS_COLOR)
            self.canMove = False
            return True
        return False

    def getCollision(self) -> list[CapsuleCollider]:
        """
        Return collision capsules for the current arm pose.

        @rtype: list[CapsuleCollider]
        """
        return self._capsulesFromFK(self.computeFK())

    def getCollisionFromPosition(self, joint_dict: dict) -> list[CapsuleCollider]:
        """
        Return collision capsules for a predicted arm pose.

        @param joint_dict: Mapping of SDK joint object to target angle.
        @type joint_dict: dict
        @rtype: list[CapsuleCollider]
        """
        return self._capsulesFromFK(self.computeFKFromJointDict(joint_dict))

    def _capsulesFromFK(self, positions: list[np.ndarray]) -> list[CapsuleCollider]:
        """
        Build the three arm capsules (upper arm, forearm, hand) from FK positions.

        @param positions: Output of L{computeFK} — 9 world-space XYZ positions.
        @type positions: list[numpy.ndarray]
        @rtype: list[CapsuleCollider]
        """
        shoulder = positions[1].tolist()
        elbow    = positions[4].tolist()
        wrist    = positions[6].tolist()
        tip      = positions[8].tolist()
        r        = config.CAPSULE_COLLISION_RADIUS
        return [
            CapsuleCollider(shoulder, elbow, r),
            CapsuleCollider(elbow,    wrist, r),
            CapsuleCollider(wrist,    tip,   r),
        ]

    def getInterpoaltionByName(self, name: str) -> "trajectory.interpolation":
        """
        Return a trajectory interpolation mode by name.

        @param name: Attribute name on trajectory.interpolation.
        @type name: str
        @return: Interpolation mode object.
        """
        return getattr(trajectory.interpolation, name)

    def safeGoto(self, joint_dict: dict, duration: float,
                 interpolation=trajectory.interpolation.linear,
                 steps: int = config.SAFE_GOTO_STEPS) -> None:
        """
        Move to a joint target with collision checking at each interpolated step.

        @param joint_dict: Mapping of SDK joint object to target angle.
        @type joint_dict: dict
        @param duration: Total movement duration in seconds.
        @type duration: float
        @param interpolation: Interpolation mode.
        @param steps: Number of collision-checked interpolation steps.
        @type steps: int
        """
        cm.MKprint(
            f"Going safely in {duration}s to {joint_dict} using {steps} collision checks.",
            self.CLASS_NAME, self.CLASS_COLOR
        )
        self._safeGoto(joint_dict, duration, interpolation, steps)

    def _safeGoto(self, joint_dict: dict, duration: float,
                  interpolation=trajectory.interpolation.linear,
                  steps: int = config.SAFE_GOTO_STEPS) -> None:
        """
        Internal safe goto implementation.

        Clamps angles, then moves in N steps with a collision check per step.
        Publishes _pendingJointDict before each check so parallel arms can
        read this arm's predicted position.

        @param joint_dict: Mapping of SDK joint object to target angle.
        @type joint_dict: dict
        @param duration: Total movement duration in seconds.
        @type duration: float
        @param interpolation: Interpolation mode.
        @param steps: Number of steps.
        @type steps: int
        """
        if not self.canMove:
            if not self.hasNotifyImpossibleMove:
                cm.MKprintSafety(
                    "Cannot safely move — please reset Reachy position.",
                    self.CLASS_NAME, self.CLASS_COLOR
                )
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
        step_duration   = duration / steps

        for i in range(1, steps + 1):
            alpha = i / steps
            interpolated = {
                joint: start_positions[joint] + alpha * (safe_target[joint] - start_positions[joint])
                for joint in safe_target
            }
            self._pendingJointDict = interpolated
            if self._checkCollision(interpolated):
                self._pendingJointDict = None
                return
            trajectory.goto(interpolated, duration=step_duration, interpolation_mode=interpolation)

        self._pendingJointDict = None

    def _debug_goto(self, joint_dict: dict, duration: float,
                    interpolation=trajectory.interpolation.linear) -> None:
        """
        Send a raw goto without any safety checks. Debug use only.

        @param joint_dict: Mapping of SDK joint object to target angle.
        @type joint_dict: dict
        @param duration: Movement duration in seconds.
        @type duration: float
        @param interpolation: Interpolation mode.
        """
        trajectory.goto(joint_dict, duration=duration, interpolation_mode=interpolation)

    def _debug_placeHandOnTable(self, duration: float = 1.0) -> None:
        """
        Move the arm to a neutral table-resting pose without collision checks.

        @param duration: Movement duration in seconds.
        @type duration: float
        """
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

    def gotoCartesianPoint(self, goalPosition: list, goalRotation: list,
                           duration: float = 0.1,
                           interpolation=trajectory.interpolation.linear) -> None:
        """
        Move the end-effector to a Cartesian position using IK.

        @param goalPosition: Target [x, y, z] position in meters.
        @type goalPosition: list[float]
        @param goalRotation: Target orientation as Euler angles [rx, ry, rz] in degrees.
        @type goalRotation: list[float]
        @param duration: Movement duration in seconds.
        @type duration: float
        @param interpolation: Interpolation mode.
        """
        IKMatrix = self._getIKMatrix(goalPosition, goalRotation)
        jointPos = self._reachyArm.inverse_kinematics(IKMatrix)
        if self.canMove:
            cm.MKprint(
                f"Going to {goalPosition} with rotation {goalRotation} in {duration}s.",
                self.CLASS_NAME, self.CLASS_COLOR
            )
        self._safeGoto(
            {joint: pos for joint, pos in zip(self._reachyArm.joints.values(), jointPos)},
            duration=duration,
            interpolation=interpolation
        )

    def changeHandAngle(self, angleEuler: float, duration: float) -> None:
        """
        Move the gripper to an absolute angle.

        @param angleEuler: Target gripper angle in degrees.
        @type angleEuler: float
        @param duration: Movement duration in seconds.
        @type duration: float
        """
        gripperName  = self._sided(config.HAND_MOTOR_NAME)
        gripperJoint = self._joints[gripperName]
        safe_angle   = self._clamp(gripperName, angleEuler, self.JOINT_GRIPPER.minAngle, self.JOINT_GRIPPER.maxAngle)
        self._safeGoto({gripperJoint: safe_angle}, duration=duration)

    def openHand(self, duration: float = 0.5) -> None:
        """
        Open the gripper fully.

        @param duration: Movement duration in seconds.
        @type duration: float
        """
        cm.MKprint(f"Opening hand in {duration}s.", self.CLASS_NAME, self.CLASS_COLOR)
        if self._armID == config.ARM_RIGHT_ID:
            self.changeHandAngle(self.JOINT_GRIPPER.minAngle, duration)
        else:
            self.changeHandAngle(self.JOINT_GRIPPER.maxAngle, duration)

    def closeHand(self, duration: float = 0.5) -> None:
        """
        Close the gripper fully.

        @param duration: Movement duration in seconds.
        @type duration: float
        """
        cm.MKprint(f"Closing hand in {duration}s.", self.CLASS_NAME, self.CLASS_COLOR)
        if self._armID == config.ARM_RIGHT_ID:
            self.changeHandAngle(self.JOINT_GRIPPER.maxAngle, duration)
        else:
            self.changeHandAngle(self.JOINT_GRIPPER.minAngle, duration)

    def _eulerToMatrix(self, anglesDeg: list) -> np.ndarray:
        """
        Convert XYZ Euler angles in degrees to a 3x3 rotation matrix.

        @param anglesDeg: Euler angles [rx, ry, rz] in degrees.
        @type anglesDeg: list[float]
        @rtype: numpy.ndarray
        """
        return R.from_euler("xyz", anglesDeg, True).as_matrix()

    def _getIKMatrix(self, goalPosition: list, goalRotationDeg: list) -> np.ndarray:
        """
        Build a 4x4 homogeneous transformation matrix for IK input.

        @param goalPosition: Target position [x, y, z].
        @type goalPosition: list[float]
        @param goalRotationDeg: Target orientation as Euler angles in degrees.
        @type goalRotationDeg: list[float]
        @rtype: numpy.ndarray
        """
        rm = self._eulerToMatrix(goalRotationDeg)
        return np.array([
            [rm[0][0], rm[0][1], rm[0][2], goalPosition[0]],
            [rm[1][0], rm[1][1], rm[1][2], goalPosition[1]],
            [rm[2][0], rm[2][1], rm[2][2], goalPosition[2]],
            [0,        0,        0,        1              ],
        ])

    def getShoulderPosition(self) -> list:
        """
        Return the shoulder position in world frame from analytical FK.

        @rtype: list[float]
        """
        return self.computeFK()[1].tolist()

    def getHandPositionFromForwardKinematicsMatrix(self, fk) -> list:
        """
        Extract the end-effector position from a 4x4 FK matrix.

        @param fk: 4x4 homogeneous transformation matrix.
        @rtype: list[float]
        """
        return [float(fk[0][3]), float(fk[1][3]), float(fk[2][3])]

    def getHandPosition(self) -> list:
        """
        Return the end-effector position from analytical FK.

        @rtype: list[float]
        """
        return self.computeFK()[8].tolist()

    def getHandPositionFromJointsPosition(self, joint_dict: dict) -> list:
        """
        Return the end-effector position for a given joint_dict via analytical FK.

        @param joint_dict: Mapping of SDK joint object to target angle.
        @type joint_dict: dict
        @rtype: list[float]
        """
        return self.computeFKFromJointDict(joint_dict)[8].tolist()

    def recordArm(self, recordDurationSeconds: float, samplingFrequencyHertz: float) -> TimeSeries:
        """
        Record arm joint positions as a time series.

        @param recordDurationSeconds: Recording duration in seconds.
        @type recordDurationSeconds: float
        @param samplingFrequencyHertz: Sampling frequency in Hz.
        @type samplingFrequencyHertz: float
        @rtype: TimeSeries
        """
        trajectories = []
        samplingTime = 1.0 / samplingFrequencyHertz
        start        = time.time()
        last_valid   = {}

        cm.MKprint(
            f"Recording {self._sided(config.ARM_NAME)} for {recordDurationSeconds}s "
            f"at {samplingFrequencyHertz}Hz.",
            self.CLASS_NAME, self.CLASS_COLOR
        )

        while (time.time() - start) < recordDurationSeconds:
            frame = {name: joint.present_position for name, joint in self._joints.items()}
            if last_valid and all(v == 0.0 for v in frame.values()):
                cm.MKprintSafety(
                    "Suspicious all-zero frame during recording — using last valid frame.",
                    self.CLASS_NAME, self.CLASS_COLOR
                )
                frame = last_valid.copy()
            else:
                last_valid = frame
            trajectories.append(frame)
            time.sleep(samplingTime)

        cm.MKprint(f"Recording done for {self._sided(config.ARM_NAME)}.", self.CLASS_NAME, self.CLASS_COLOR)
        return TimeSeries(
            samplingFrequencyHertz,
            recordDurationSeconds,
            trajectories,
            [self._armID == config.ARM_RIGHT_ID, self._armID == config.ARM_LEFT_ID, 0]
        )

    def playArmRecord(self, record: TimeSeries, startDuration: float = 3.0,
                      collisionCheckInterval: int = 5) -> None:
        """
        Replay a recorded arm time series with periodic collision checks.

        @param record: Time series to replay.
        @type record: TimeSeries
        @param startDuration: Duration to move to the first frame, in seconds.
        @type startDuration: float
        @param collisionCheckInterval: Number of frames between collision checks.
            Set to 0 to disable.
        @type collisionCheckInterval: int
        """
        if not self.canMove:
            if not self.hasNotifyImpossibleMove:
                cm.MKprintSafety(
                    "Cannot safely move — please reset Reachy position.",
                    self.CLASS_NAME, self.CLASS_COLOR
                )
                self.hasNotifyImpossibleMove = True
            return

        firstPoint = {
            self._joints[m]: pos
            for m, pos in record.jointPosition[0].items()
            if m in self._joints
        }
        samplingTime = 1.0 / record.samplingFrequency

        cm.MKprintDebug(
            f"samplingTime={samplingTime} samplingFrequency={record.samplingFrequency}",
            self.CLASS_NAME, self.CLASS_COLOR
        )
        cm.MKprint(f"Playing record for {self._sided(config.ARM_NAME)}.", self.CLASS_NAME, self.CLASS_COLOR)

        self._safeGoto(firstPoint, duration=startDuration)

        for i, jointsPositions in enumerate(record.jointPosition):
            t_start  = time.time()
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
