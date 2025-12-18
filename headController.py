from reachy_sdk import trajectory, ReachySDK
import time
import consoleManager as cm
import timeSerieManager as ts

class ReachyDisk():
    def __init__(self, maxAngleEuler : float, minAngleEuler : float):
        self.maxAngle = maxAngleEuler
        self.minAngle = minAngleEuler
    
class ReachyHead():

    #name
    ANTENNA_MOTOR_NAME : str = ["l_antenna", "r_antenna"]
    DISK_MOTOR_NAME : str = ["neck_roll", "neck_pitch", "neck_yaw"]
    HEAD_MOTOR_NAME : list = DISK_MOTOR_NAME + ANTENNA_MOTOR_NAME
    #console
    CLASS_NAME : str = "Reachy head"
    CLASS_COLOR : str = cm.Color.YELLOW

    DISK_NECK_ROLL : ReachyDisk = ReachyDisk(60, -60)
    DISK_NECK_PITCH : ReachyDisk = ReachyDisk(60, -60)
    DISK_NECK_YAW : ReachyDisk = ReachyDisk(360, 0)

    def __init__(self, reachy : ReachySDK):
        self._reachyHead = reachy.head
        self._disks = self._setupDisks()

    def _setupDisks(self) -> dict:
        r : list = {}

        for jointName in ReachyHead.HEAD_MOTOR_NAME:
            joint = getattr(self._reachyHead, jointName)
            r[jointName] = joint
        
        return r
    
    def lookAt(self, degAngles : list, duration : float = 1) -> None:
        self._reachyHead.look_at(x=degAngles[0], y=degAngles[1], z=degAngles[2], duration=duration)

    def recordHead(self, recordDurationSeconds : float, samplingFrequencyHertz : float) -> "ts.TimeSeries":
        """
        record all head position during recordDurationSeconds second(s) with a sampling frequency of samplingFrequencySeconds
        PARAMETER recordDurationSeconds TYPE float, samplingFrequencySeconds TYPE float
        RETURN ts.TimeSeries
        """
        trajectories = []
        samplingTime : float = 1.0 / samplingFrequencyHertz
        start = time.time()

        cm.MKprint("start recording head joints for " + str(recordDurationSeconds) + "s with a sampling frequency of " + str(samplingFrequencyHertz) + "Hz.", ReachyHead.CLASS_NAME, ReachyHead.CLASS_COLOR)

        while (time.time() - start) < recordDurationSeconds:
            current_point = {name: joint.present_position for name, joint in self._disks.items()}
            trajectories.append(current_point)

            time.sleep(samplingTime)
        
        cm.MKprint("records for head joints done !", ReachyHead.CLASS_NAME, ReachyHead.CLASS_COLOR)


        return ts.TimeSeries(samplingFrequencyHertz, recordDurationSeconds, trajectories)

    def playHeadRecord(self, record : "ts.TimeSeries", startDuration : float = 3.0) -> None:
        """
        play record for head, you need to specify the semplingFrequencySeconds. startDuration is used to set the time reachy will take to go to the start position
        PARAMETER record TYPE ts.TimeSeries, startDuration TYPE float
        RETURN None
        """
        def firstPoint() -> dict:
            r : dict = {}
            for m in record.jointPosition[0].keys():
                if m in self._disks.keys():
                    r[self._disks[m]] = record.jointPosition[0][m]
            return r
        
        firstPoint = firstPoint()

        samplingTime : float = 1.0/record.samplingFrequency

        cm.MKprint("start playing records for head joints with a sampling frequency of " + str(samplingTime) + "hz.", ReachyHead.CLASS_NAME, ReachyHead.CLASS_COLOR)


        trajectory.goto(firstPoint, duration=startDuration)

        for jointsPositions in record.jointPosition:
            for joint_name, pos in jointsPositions.items():
                if joint_name in self._disks.keys():
                    self._disks[joint_name].goal_position = pos
            time.sleep(samplingTime)

        return None
    
if __name__ == "__main__":
    reachy = ReachySDK(host='localhost')
    head = ReachyHead(reachy)
    head.lookAt([3, 2, 0])
    a = head.recordHead(1, 20)
    head.playHeadRecord(a, 20)
