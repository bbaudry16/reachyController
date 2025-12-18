from reachy_sdk import trajectory, ReachySDK
import time
import consoleManager as cm

class ReachyDisk():
    def __init__(self, maxAngleEuler : float, minAngleEuler : float):
        self.maxAngle = maxAngleEuler
        self.minAngle = minAngleEuler
    
class ReachyHead():

    #name
    ANTENNA_MOTOR_NAME : str = ["l_antenna", "r_antenna"]
    HEAD_MOTOR_NAME : list = ["neck_roll", "neck_pitch", "neck_yaw"] + ANTENNA_MOTOR_NAME
    #console
    CLASS_NAME : str = "Reachy head"
    CLASS_COLOR : str = cm.Color.YELLOW
    
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

    def recordHead(self, recordDurationSeconds : float, samplingFrequencyHertz : float) -> dict:
        """
        record all head position during recordDurationSeconds second(s) with a sampling frequency of samplingFrequencySeconds
        PARAMETER recordDurationSeconds TYPE float, samplingFrequencySeconds TYPE float
        RETURN dict
        """
        trajectories = []
        samplingTime : float = 1.0 / samplingFrequencyHertz
        start = time.time()

        cm.MKprint("start recording the head for " + str(recordDurationSeconds) + "s with a sampling frequency of " + str(samplingFrequencyHertz) + "Hz.", ReachyHead.CLASS_NAME, ReachyHead.CLASS_COLOR)

        while (time.time() - start) < recordDurationSeconds:
            current_point = {name: joint.present_position for name, joint in self._disks.items()}
            trajectories.append(current_point)

            time.sleep(samplingTime)
        
        cm.MKprint("records for head done !", ReachyHead.CLASS_NAME, ReachyHead.CLASS_COLOR)


        return {"samplingFrequency" : samplingFrequencyHertz, "recordDuration" : recordDurationSeconds, "startTime" : start, "diskPosition" : trajectories}
    
    def playHeadRecord(self, record : list, samplingFrequencyHertz : float, startDuration : float = 3.0) -> None:
        """
        play record from an arm, you need to specify the semplingFrequencySeconds. startDuration is used to set the time reachy will take to go to the start position
        PARAMETER record TYPE list, samplingFrequencySeconds TYPE float, startDuration TYPE float
        RETURN None
        """
        firstPoint = { self._disks[name]: pos for name, pos in record[0].items() }

        samplingTime : float = 1.0/samplingFrequencyHertz

        cm.MKprint("start playing records for head with a sampling frequency of " + str(samplingTime) + "s.", ReachyHead.CLASS_NAME, ReachyHead.CLASS_COLOR)


        trajectory.goto(firstPoint, duration=startDuration)

        for jointsPositions in record:
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
    print(a)
    head.playHeadRecord(a["diskPosition"], 20)
