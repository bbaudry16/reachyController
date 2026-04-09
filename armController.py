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

    # ─── Collision checks ──────────────────────────────────────────────────────

    def _collideWithTable(self, joint_dict: dict) -> bool:
        return self.getHandPositionFromJointsPosition(joint_dict)[2] <= config.TABLE_Z_COORD

    def _collideWithOtherPart(self, joint_dict: dict) -> bool:
        if self._collisionManager is not None:
            return not self._collisionManager.askValidMovement(self._armID, joint_dict)
        return False

    def _checkCollision(self, joint_dict: dict) -> bool:
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
        self.changeHandAngle(self.JOINT_GRIPPER.minAngle, duration)

    def closeHand(self, duration: float = 0.5) -> None:
        self.changeHandAngle(self.JOINT_GRIPPER.maxAngle, duration)

    # ─── Kinematics ────────────────────────────────────────────────────────────

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
        """
        Shoulder position in world frame (origin = torso center).
        Right arm: x = -ORIGIN_TO_SHOULDER
        Left arm:  x = +ORIGIN_TO_SHOULDER
        """
        x = -config.ORIGIN_TO_SHOULDER
        if self._armID == config.ARM_LEFT_ID:
            x *= -1
        return [x, 0, 0]

    def getHandPositionFromForwardKinematicsMatrix(self, fk) -> list:
        """Extract XYZ translation from a 4x4 FK matrix."""
        return [float(fk[0][3]), float(fk[1][3]), float(fk[2][3])]

    def getHandPosition(self) -> list:
        """
        Hand position in world frame.
        The SDK FK has its origin at the shoulder with no global rotation offset,
        so the returned coordinates are directly in world frame.
        """
        return self.getHandPositionFromForwardKinematicsMatrix(
            self._reachyArm.forward_kinematics()
        )

    def getHandPositionFromJointsPosition(self, joint_dict: dict) -> list:
        """
        Hand position in world frame from a joint dict.
        joint_dict keys must be joint SDK objects (as returned by self._joints).
        Only the 7 arm joints are used (gripper and timestamp are ignored).
        """
        ordered = ["shoulder_pitch", "shoulder_roll", "arm_yaw",
                   "elbow_pitch", "forearm_yaw", "wrist_pitch", "wrist_roll"]
        jointPos = [joint_dict[self._joints[self._sided(j)]] for j in ordered]
        fk = self._reachyArm.forward_kinematics(jointPos)
        return self.getHandPositionFromForwardKinematicsMatrix(fk)

    # ─── Collision shapes ──────────────────────────────────────────────────────
    #
    # The arm is modelled as two capsules:
    #   capsule 1 : shoulder → midpoint  (upper arm approximation)
    #   capsule 2 : midpoint → hand      (forearm approximation)
    #
    # The midpoint is the geometric average of shoulder and hand positions.
    # This avoids manual elbow FK which proved unreliable due to unknown
    # SDK joint-frame conventions — the SDK has no partial FK support.

    def _midpoint(self, a: list, b: list) -> list:
        return [(a[i] + b[i]) / 2.0 for i in range(3)]

    def getCollision(self) -> list[CapsuleCollider]:
        shoulder = self.getShoulderPosition()
        hand     = self.getHandPosition()
        mid      = self._midpoint(shoulder, hand)
        return [
            CapsuleCollider(shoulder, mid,  config.CAPSULE_COLLISION_RADIUS),
            CapsuleCollider(mid,      hand, config.CAPSULE_COLLISION_RADIUS),
        ]

    def getCollisionFromPosition(self, joint_dict: dict) -> list[CapsuleCollider]:
        shoulder = self.getShoulderPosition()
        hand     = self.getHandPositionFromJointsPosition(joint_dict)
        mid      = self._midpoint(shoulder, hand)
        return [
            CapsuleCollider(shoulder, mid,  config.CAPSULE_COLLISION_RADIUS),
            CapsuleCollider(mid,      hand, config.CAPSULE_COLLISION_RADIUS),
        ]

    # ─── Record / Play ─────────────────────────────────────────────────────────

    def recordArm(self, recordDurationSeconds: float, samplingFrequencyHertz: float) -> TimeSeries:
        trajectories = []
        samplingTime = 1.0 / samplingFrequencyHertz
        start        = time.time()

        cm.MKprint(
            f"Recording {self._sided(config.ARM_NAME)} for {recordDurationSeconds}s at {samplingFrequencyHertz}Hz.",
            self.CLASS_NAME, self.CLASS_COLOR
        )

        while (time.time() - start) < recordDurationSeconds:
            frame = {name: joint.present_position for name, joint in self._joints.items()}
            frame["timestamp"] = time.time()
            trajectories.append(frame)
            time.sleep(samplingTime)

        cm.MKprint(f"Recording done for {self._sided(config.ARM_NAME)}.", self.CLASS_NAME, self.CLASS_COLOR)
        return TimeSeries(samplingFrequencyHertz, recordDurationSeconds, trajectories)

    def playArmRecord(self, record: TimeSeries, startDuration: float = 3.0, collisionCheckInterval: int = 0) -> None:
        """
        Play a recorded arm trajectory.
        collisionCheckInterval: run collision check every N frames (default 5).
        Collision is not checked on frame 0 — the starting position is assumed
        safe since it was physically reached during recording.
        """
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


            for name, pos in jointsPositions.items():
                if i > 0 and i % collisionCheckInterval == 0:
                    if self._checkCollision(safe_step):
                        return
                
                if name in self._joints:
                    self._joints[name].goal_position = pos

            elapsed = time.time() - t_start
            remaining = samplingTime - elapsed
            if remaining > 0:
                time.sleep(remaining)