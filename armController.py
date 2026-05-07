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


class ReachyJoint:
    """
    describe reachy joint with a max and a min angle
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

        self._collisionManager  = collisionManager
        if self._collisionManager is None:
            cm.MKprintSafety("No collision manager set, inter-part collision will be ignored.", self.CLASS_NAME, self.CLASS_COLOR)

        self.canMove                 : bool = True
        self.hasNotifyImpossibleMove : bool = False

    # ─── Setup ─────────────────────────────────────────────────────────────────

    def _setupConstraints(self) -> None:
        """
        setup constraint for shoulder, wrist and gripper
        PARAMETER None
        RETURN None
        """
        if self._armID == config.ARM_LEFT_ID:
            self.JOINT_SHOULDER_ROLL = ReachyJoint(180.0, -10.0)
            self.JOINT_WRIST_ROLL    = ReachyJoint(55, -35)
            self.JOINT_GRIPPER       = ReachyJoint(69, -20)
        return None

    def _setupJoints(self) -> dict:
        """
        setup joint in __init__
        PARAMETER None
        RETURN dict
        """
        r = {}
        for name in config.ARM_MOTOR_NAME:
            sided = self._sided(name)
            r[sided] = getattr(self._reachyArm, sided)
        return r

    def _setupJointConstraints(self) -> dict:
        """
        setup all joint constraints in __init__
        PARAMETER None
        RETURN dict
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

    # ─── Public setters ────────────────────────────────────────────────────────

    def setCollisionManager(self, collisionManager) -> None:
        """
        set a collision manager
        PARAMETER collisionManager
        RETURN None
        """
        self._collisionManager = collisionManager
        cm.MKprintSafety("Collision manager set — inter-part collision is now active.", self.CLASS_NAME, self.CLASS_COLOR)
        return None

    def getArmId(self) -> str:
        """
        return the arm id
        PARAMETER None
        RETURN str
        """
        return self._armID

    def resetCanMove(self) -> None:
        """
        reset capability to move (use after an obstacle)
        PARAMETER None
        RETURN None
        """
        self.canMove = True
        self.hasNotifyImpossibleMove = False

    # ─── Internal helpers ──────────────────────────────────────────────────────

    def _sided(self, name: str) -> str:
        """
        return a sided string, example : "gripper" -> "l_gripper"
        PARAMETER name : str
        RETURN str
        """
        return self._armID + "_" + name

    def _clamp(self, jointName: str, value: float, min_v: float, max_v: float) -> float:
        """
        clamp a value between min and max
        PARAMETER value : float, min_v : float, max_v : float
        RETURN float
        """
        r = max(min(value, max_v), min_v)
        if value < min_v or value > max_v:
            cm.MKprintSafety(f"{jointName} clamped to {r}", self.CLASS_NAME, self.CLASS_COLOR)
        return r

    # ─── Collision checks ──────────────────────────────────────────────────────

    def _collideWithTable(self, joint_dict: dict) -> bool:
        """
        return true if the hand is below the table
        PARAMETER joint_dict : dict
        RETURN bool
        """
        return self.getHandPositionFromJointsPosition(joint_dict)[2] <= config.TABLE_Z_COORD

    def _collideWithOtherPart(self, joint_dict: dict) -> bool:
        """
        return if there is a collision with other part of the robot
        PARAMETER joint_dict : dict
        RETURN bool
        """
        if self._collisionManager is not None:
            return not self._collisionManager.askValidMovement(self._armID, joint_dict)
        return False

    def _checkCollision(self, joint_dict: dict) -> bool:
        """
        check all types of collision and freeze the robot if one is detected
        PARAMETER joint_dict : dict
        RETURN bool
        """
        if self._collideWithTable(joint_dict):
            cm.MKprintSafety("Collision with table!", self.CLASS_NAME, self.CLASS_COLOR)
            self.canMove = False
            return True
        if self._collideWithOtherPart(joint_dict):
            cm.MKprintSafety("Collision with another part!", self.CLASS_NAME, self.CLASS_COLOR)
            self.canMove = False
            return True
        return False

    # ─── Motion ────────────────────────────────────────────────────────────────

    def _safeGoto(self, joint_dict: dict, duration: float, interpolation=trajectory.interpolation.linear, steps: int = config.SAFE_GOTO_STEPS) -> None:
        """
        a goto that takes into account collision and clamping
        PARAMETER joint_dict : dict, duration : float, interpolation : trajectory.interpolation, steps : int
        RETURN None
        """
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
            if self._checkCollision(interpolated):
                return
            trajectory.goto(interpolated, duration=step_duration, interpolation_mode=interpolation)

        return None

    def _debug_goto(self, joint_dict: dict, duration: float, interpolation=trajectory.interpolation.linear) -> None:
        """Not safe — debug use only."""
        trajectory.goto(joint_dict, duration=duration, interpolation_mode=interpolation)

    def _debug_placeHandOnTable(self, duration: float = 1.0) -> None:
        """
        do not use unless debugging
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

    def gotoCartesianPoint(self, goalPosition: list, goalRotation: list, duration: float = 0.1, interpolation=trajectory.interpolation.linear) -> None:
        """
        move the end to a cartesian coordinate with a position, a rotation, a duration and an interpolation method
        PARAMETER goalPosition : list, goalRotation : list, duration : float, interpolation : trajectory.interpolation
        RETURN None
        """
        IKMatrix = self._getIKMatrix(goalPosition, goalRotation)
        jointPos = self._reachyArm.inverse_kinematics(IKMatrix)
        if self.canMove:
            cm.MKprint(f"Going to {goalPosition} with rotation {goalRotation} in {duration}s.", self.CLASS_NAME, self.CLASS_COLOR)
        self._safeGoto(
            {joint: pos for joint, pos in zip(self._reachyArm.joints.values(), jointPos)},
            duration=duration,
            interpolation=interpolation
        )
        return None

    def changeHandAngle(self, angleEuler: float, duration: float) -> None:
        """
        change gripper angle with a duration
        PARAMETER angleEuler : float, duration : float
        RETURN None
        """
        gripperName  = self._sided(config.HAND_MOTOR_NAME)
        gripperJoint = self._joints[gripperName]
        safe_angle   = self._clamp(gripperName, angleEuler, self.JOINT_GRIPPER.minAngle, self.JOINT_GRIPPER.maxAngle)
        self._safeGoto({gripperJoint: safe_angle}, duration=duration)
        return None

    def openHand(self, duration: float = 0.5) -> None:
        """
        open the hand with a duration
        PARAMETER duration : float
        RETURN None
        """
        self.changeHandAngle(self.JOINT_GRIPPER.minAngle, duration)
        return None

    def closeHand(self, duration: float = 0.5) -> None:
        """
        close the hand with a duration
        PARAMETER duration : float
        RETURN None
        """
        self.changeHandAngle(self.JOINT_GRIPPER.maxAngle, duration)
        return None

    # ─── Kinematics ────────────────────────────────────────────────────────────

    def _eulerToMatrix(self, anglesDeg: list):
        """
        transform a euler angle list to a matrix
        PARAMETER anglesDeg : list
        RETURN list
        """
        return R.from_euler("xyz", anglesDeg, True).as_matrix()

    def _getIKMatrix(self, goalPosition: list, goalRotationDeg: list):
        """
        return the IK matrix used in reachy sdk for IK
        PARAMETER goalPosition : list, goalRotationDeg : float
        RETURN np.array
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
        Shoulder position in world frame (origin = torso center, at shoulder height).
        Right arm: x = -ORIGIN_TO_SHOULDER
        Left arm:  x = +ORIGIN_TO_SHOULDER
        PARAMETER None
        RETURN list [x, y, z]
        """
        x = -config.ORIGIN_TO_SHOULDER
        if self._armID == config.ARM_LEFT_ID:
            x *= -1
        return [x, 0, 0]

    def getHandPositionFromForwardKinematicsMatrix(self, fk) -> list:
        """
        Extract XYZ world coordinates from a 4x4 FK matrix.
        The SDK forward_kinematics() returns coords in world frame directly.
        PARAMETER fk : np.array 4x4
        RETURN list [x, y, z]
        """
        return [float(fk[0][3]), float(fk[1][3]), float(fk[2][3])]

    def getHandPosition(self) -> list:
        """
        Hand position in world frame from current joint positions.
        PARAMETER None
        RETURN list [x, y, z]
        """
        return self.getHandPositionFromForwardKinematicsMatrix(
            self._reachyArm.forward_kinematics()
        )

    def getHandPositionFromJointsPosition(self, joint_dict: dict) -> list:
        """
        Hand position in world frame from a joint dict.
        joint_dict keys must be joint SDK objects (as returned by self._joints).
        Only the 7 arm joints are used (gripper is ignored).
        PARAMETER joint_dict : dict
        RETURN list [x, y, z]
        """
        ordered = ["shoulder_pitch", "shoulder_roll", "arm_yaw",
                   "elbow_pitch", "forearm_yaw", "wrist_pitch", "wrist_roll"]
        jointPos = [joint_dict[self._joints[self._sided(j)]] for j in ordered]
        fk = self._reachyArm.forward_kinematics(jointPos)
        return self.getHandPositionFromForwardKinematicsMatrix(fk)

    # ─── Collision shapes ──────────────────────────────────────────────────────
    #
    # The arm is modelled as two capsules built from the FK segment shoulder→hand:
    #   capsule 0 : shoulder  → midpoint   (upper arm approximation)
    #   capsule 1 : midpoint  → hand       (forearm approximation)
    #
    # midpoint = geometric average of shoulder and hand (FK world coords).
    #
    # WHY NOT USE A REAL ELBOW:
    # The SDK forward_kinematics() always computes up to the end-effector —
    # passing 0 for distal joints does NOT stop at the elbow.
    # Manual trigonometry gives the correct elbow position in world space for
    # the LEFT arm, but produces an inconsistent result for the RIGHT arm due
    # to an unknown internal frame convention difference in the Reachy SDK.
    # Empirically, with the right arm in the neutral pose (elbow_pitch=-90°),
    # the trig elbow is at X=-0.19 while the FK hand is at X=+0.36, yielding
    # a forearm capsule of 0.59m instead of 0.25m — the midpoint heuristic
    # avoids this inconsistency by staying within the FK coordinate space.
    #
    # TORSO CHECK: only capsule 1 (forearm / midpoint→hand) is checked against
    # the torso. Capsule 0 (shoulder→midpoint) crosses near the torso axis on
    # the right arm in safe poses due to the geometry of the FK segment, and is
    # excluded from torso checks to prevent false positives.

    def _midpoint(self, a: list, b: list) -> list:
        """
        return the midpoint between two points
        PARAMETER a : list, b : list
        RETURN list
        """
        return [(a[i] + b[i]) / 2.0 for i in range(3)]

    def getCollision(self) -> list[CapsuleCollider]:
        """
        Return collision capsules for the current arm pose (live joints).
        PARAMETER None
        RETURN list[CapsuleCollider]
        """
        shoulder = self.getShoulderPosition()
        hand     = self.getHandPosition()
        mid      = self._midpoint(shoulder, hand)
        return [
            CapsuleCollider(shoulder, mid,  config.CAPSULE_COLLISION_RADIUS),
            CapsuleCollider(mid,      hand, config.CAPSULE_COLLISION_RADIUS),
        ]

    def getCollisionFromPosition(self, joint_dict: dict) -> list[CapsuleCollider]:
        """
        Return collision capsules for a given joint dict (predictive check).
        PARAMETER joint_dict : dict  (keys = joint SDK objects)
        RETURN list[CapsuleCollider]
        """
        shoulder = self.getShoulderPosition()
        hand     = self.getHandPositionFromJointsPosition(joint_dict)
        mid      = self._midpoint(shoulder, hand)
        return [
            CapsuleCollider(shoulder, mid,  config.CAPSULE_COLLISION_RADIUS),
            CapsuleCollider(mid,      hand, config.CAPSULE_COLLISION_RADIUS),
        ]

    # ─── Record / Play ─────────────────────────────────────────────────────────

    def recordArm(self, recordDurationSeconds: float, samplingFrequencyHertz: float) -> TimeSeries:
        """
        record the arm into a time series
        PARAMETER recordDurationSeconds : float, samplingFrequencyHertz : float
        RETURN TimeSeries
        """
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
        return TimeSeries(samplingFrequencyHertz, recordDurationSeconds, trajectories)

    def playArmRecord(self, record: TimeSeries, startDuration: float = 3.0, collisionCheckInterval: int = 5) -> None:
        """
        Play a recorded arm trajectory.
        collisionCheckInterval : check collision every N frames (0 = disabled).
        Frame 0 is never checked — starting position is assumed safe since it
        was physically reached during the original recording.
        PARAMETER record : TimeSeries, startDuration : float, collisionCheckInterval : int
        RETURN None
        """
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

            # Build joint_dict with SDK joint objects as keys (required by collision checks)
            safe_step = {
                self._joints[name]: pos
                for name, pos in jointsPositions.items()
                if name in self._joints
            }

            # ── Collision check once per frame, outside the joints loop ──────
            # collisionCheckInterval == 0 disables collision checking entirely.
            if i > 0 and collisionCheckInterval > 0 and i % collisionCheckInterval == 0:
                if self._checkCollision(safe_step):
                    return

            # ── Apply positions to joints ─────────────────────────────────────
            for name, pos in jointsPositions.items():
                if name in self._joints:
                    self._joints[name].goal_position = pos

            elapsed   = time.time() - t_start
            remaining = samplingTime - elapsed
            if remaining > 0:
                time.sleep(remaining)