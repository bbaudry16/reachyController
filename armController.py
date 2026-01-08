import numpy as np
from reachy_sdk import trajectory, ReachySDK
from scipy.spatial.transform import Rotation as R
import consoleManager as cm
import time
import timeSerieManager as ts

class ReachyJoint():
    def __init__(self, maxAngleEuler : float, minAngleEuler : float):
        self.maxAngle = maxAngleEuler
        self.minAngle = minAngleEuler

class ReachyArm():
    #contraint
    JOINT_SHOULDER_PITCH = ReachyJoint(90.0, -150.0)
    JOINT_SHOULDER_ROLL = ReachyJoint(10.0, -180.0)
    JOINT_ARM_YAW = ReachyJoint(90.0, -90.0)
    JOINT_ELBOW_PITCH = ReachyJoint(0.0, -125.0)
    JOINT_FOREARM_YAW = ReachyJoint(100.0, -100.0)
    JOINT_WRIST_PITCH = ReachyJoint(45.0, -45.0)
    JOINT_WRIST_ROLL = ReachyJoint(35, -55)
    JOINT_GRIPPER = ReachyJoint(20.0, -69.0)
    #name
    HAND_MOTOR_NAME : str = "gripper"
    SHOULDER_MOTOR_NAME : list = ["shoulder_pitch", "shoulder_roll", "arm_yaw"]
    ELBOW_MOTOR_NAME : list = ["elbow_pitch", "forearm_yaw"]
    FOREARM_MOTOR_NAME : list = ["wrist_pitch", "wrist_roll", HAND_MOTOR_NAME]
    ARM_MOTOR_NAME : list = SHOULDER_MOTOR_NAME + ELBOW_MOTOR_NAME + FOREARM_MOTOR_NAME
    ARM_NAME : str = "arm"
    ARM_LEFT_ID : str = "l"
    #console manager
    CLASS_NAME : str = "Reachy arm"
    CLASS_COLOR : str = cm.Color.CYAN

    def __init__(self, _reachy : ReachySDK, _armID : str) -> None:
        
        self._armID : str = _armID
        self._setupConstraints()
        
        self._reachyArm = getattr(_reachy, self._getNameByArmSide(ReachyArm.ARM_NAME))
        self._joints : dict = self._setupJoints()

        self._joint_constraints = {
        self._getNameByArmSide("shoulder_pitch"): self.JOINT_SHOULDER_PITCH,
        self._getNameByArmSide("shoulder_roll"): self.JOINT_SHOULDER_ROLL,
        self._getNameByArmSide("arm_yaw"): self.JOINT_ARM_YAW,
        self._getNameByArmSide("elbow_pitch"): self.JOINT_ELBOW_PITCH,
        self._getNameByArmSide("forearm_yaw"): self.JOINT_FOREARM_YAW,
        self._getNameByArmSide("wrist_pitch"): self.JOINT_WRIST_PITCH,
        self._getNameByArmSide("wrist_roll"): self.JOINT_WRIST_ROLL,
        self._getNameByArmSide("gripper"): self.JOINT_GRIPPER,
        }

        return None

    def _setupConstraints(self) -> None:
        if self._armID == ReachyArm.ARM_LEFT_ID:
            self.JOINT_SHOULDER_ROLL = ReachyJoint(-10.0, 180.0)
            self.JOINT_WRIST_ROLL = ReachyJoint(-35, 55)
            self.JOINT_GRIPPER = ReachyJoint(-20.0, 69)


    def _setupJoints(self) -> dict:
        r : dict = {}

        for jointName in ReachyArm.ARM_MOTOR_NAME:
            _jointsidedName : str = self._getNameByArmSide(jointName)  
            joint = getattr(self._reachyArm, _jointsidedName)
            r[_jointsidedName] = joint
        
        return r
    
    def _clamp(self, jointName : str, value: float, min_v: float, max_v: float) -> float:
        r : float = max(min(value, max_v), min_v)
        if value < min_v or value > max_v:
            cm.MKprint(cm.Color.RED + f"[SAFETY] {jointName} clamped to {r}" + cm.Color.RESET, ReachyArm.CLASS_NAME, ReachyArm.CLASS_COLOR)
        return r
    
    def _safeGoto(self, joint_dict : dict, duration : float, interpolation):
        safe_dict = {}
        for joint, pos in joint_dict.items():
            name = joint.name
            if name in self._joint_constraints:
                limits = self._joint_constraints[name]
                pos = self._clamp(name, pos, limits.minAngle, limits.maxAngle)
            safe_dict[joint] = pos

        trajectory.goto(safe_dict, duration=duration, interpolation_mode=interpolation)


    def _getNameByArmSide(self, name : str) -> str:
        return self._armID + "_" + name
    

    def _eulerToMatrix(self, anglesDeg : list) -> list:
        """
        return an euler rotation array into a rotation matrix
        PARAMETER angles TYPE list[float]
        RETURN list[list[float]]
        """
        return R.from_euler("xyz", anglesDeg, True).as_matrix()


    def _getIKMatrix(self, goalPosition : list, goalRotationDeg : list) -> list:
        """
        return the matrix needed to perform an IK on reachy
        PARAMETER goalPosition TYPE list[float] (vect 3), goalRotationDeg TYPE list[float] (vect 3)
        RETURN list[list[float]] (as an np.array, uniform matrix) 
        """
        rotationMatrix : list = self._eulerToMatrix(goalRotationDeg)

        return np.array([
            [ rotationMatrix[0][0], rotationMatrix[0][1], rotationMatrix[0][2], goalPosition[0] ],
            [ rotationMatrix[1][0], rotationMatrix[1][1], rotationMatrix[1][2], goalPosition[1] ],
            [ rotationMatrix[2][0], rotationMatrix[2][1], rotationMatrix[2][2], goalPosition[2] ],
            [ 0, 0, 0, 1 ]
        ])
    
   
    def gotoCartesianPoint(self, goalPosition : list, goalRotation : list, duration : float = 0.1, interpolation = trajectory.interpolation.linear) -> None:
        IKMatrix : list = self._getIKMatrix(goalPosition, goalRotation)
        jointPos = self._reachyArm.inverse_kinematics(IKMatrix)
        
        
        
        cm.MKprint("Going to " + str(goalPosition) + " with rotation " + str(goalRotation) + " in " + str(duration) + "s.", ReachyArm.CLASS_NAME, ReachyArm.CLASS_COLOR)
        self._safeGoto({joint: pos for joint,pos in zip(self._reachyArm.joints.values(), jointPos)}, duration=duration, interpolation=interpolation)

        return None


    def changeHandAngle(self, angleEuler : float, duration : float) -> None:
        gripperJointName = self._getNameByArmSide(ReachyArm.HAND_MOTOR_NAME)
        gripperJoint = self._joints[gripperJointName]
        
        safe_angle = self._clamp(gripperJointName, angleEuler, ReachyArm.JOINT_GRIPPER.minAngle, ReachyArm.JOINT_GRIPPER.maxAngle)

        self._safeGoto({gripperJoint: safe_angle}, duration=duration)

    def openHand(self, duration = 0.5) -> None:
        self.changeHandAngle(self.JOINT_GRIPPER.minAngle, duration)

    def closeHand(self, duration = 0.5) -> None:
        self.changeHandAngle(self.JOINT_GRIPPER.maxAngle, duration)


    def recordArm(self, recordDurationSeconds : float, samplingFrequencyHertz : float) -> "ts.TimeSeries":
        """
        record all arm position during recordDurationSeconds second(s) with a sampling frequency of samplingFrequencySeconds
        PARAMETER recordDurationSeconds TYPE float, samplingFrequencySeconds TYPE float
        RETURN ts.TimeSeries
        """
        trajectories = []
        samplingTime : float = 1.0 / samplingFrequencyHertz
        start = time.time()

        cm.MKprint("start recording " + self._getNameByArmSide(ReachyArm.ARM_NAME) + " for " + str(recordDurationSeconds) + "s with a sampling frequency of " + str(samplingFrequencyHertz) + "Hz.", ReachyArm.CLASS_NAME, ReachyArm.CLASS_COLOR)

        while (time.time() - start) < recordDurationSeconds:
            current_point = {name: joint.present_position for name, joint in self._joints.items()}
            trajectories.append(current_point)

            time.sleep(samplingTime)
        
        cm.MKprint("records for arm " + self._getNameByArmSide(ReachyArm.ARM_NAME) + " done !", ReachyArm.CLASS_NAME, ReachyArm.CLASS_COLOR)


        return ts.TimeSeries(samplingFrequencyHertz, recordDurationSeconds, trajectories)
    
    def playArmRecord(self, record: "ts.TimeSeries", startDuration: float = 3.0) -> None:
        """
        play record from an arm, you need to specify the semplingFrequencySeconds. startDuration is used to set the time reachy will take to go to the start position
        PARAMETER record TYPE ts.TimeSeries, startDuration TYPE float
        RETURN None
        """
        def firstPoint():
            return {
                self._joints[m]: pos
                for m, pos in record.jointPosition[0].items()
                if m in self._joints
            }

        samplingTime = 1.0 / record.samplingFrequency

        cm.MKprint(
            "start playing records for arm "
            + self._getNameByArmSide(ReachyArm.ARM_NAME),
            ReachyArm.CLASS_NAME,
            ReachyArm.CLASS_COLOR
        )

        self._safeGoto(firstPoint(), duration=startDuration)

        for jointsPositions in record.jointPosition:
            safe_step = {}

            for joint_name, pos in jointsPositions.items():
                if joint_name in self._joints:
                    joint = self._joints[joint_name]
                    safe_step[joint] = pos

            self._safeGoto(
                safe_step,
                duration=samplingTime,
                interpolation=trajectory.interpolation.linear
            )

        return None
    
if __name__ == "__main__":
    reachy = ReachySDK(host='localhost')
    arm : ReachyArm = ReachyArm(reachy, "l")
    arm.gotoCartesianPoint([1, 0, 0], [0, -90, 0], 5)
    arm.openHand()
    arm.gotoCartesianPoint([0, 1, 0], [0, -90, 0], 5)
    records = arm.recordArm(5, 20)
    arm.gotoCartesianPoint([1, 0, 0], [0, -90, 0], 5)
    arm.playArmRecord(records)
