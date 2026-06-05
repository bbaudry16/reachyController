from reachy_sdk import trajectory, ReachySDK
import time

from . import config
from . import reachyPart as rp
from . import consoleManager as cm
from .timeSeries import TimeSeries
from scipy.spatial.transform import Rotation as R
import numpy as np
from reachy_sdk import trajectory

class ReachyDisk:
    def __init__(self, maxAngleEuler: float, minAngleEuler: float):
        self.maxAngle = maxAngleEuler
        self.minAngle = minAngleEuler


class ReachyHead(rp.ReachyPart):

    DISK_NECK_ROLL  : ReachyDisk = ReachyDisk(60,  -60)
    DISK_NECK_PITCH : ReachyDisk = ReachyDisk(60,  -60)
    DISK_NECK_YAW   : ReachyDisk = ReachyDisk(360,   0)

    CLASS_NAME  : str = "Reachy head"
    CLASS_COLOR : str = cm.Color.BRIGHT_BLUE

    def __init__(self, reachy: ReachySDK):
        self._reachyHead = reachy.head
        self._disks      = self._setupDisks()

    def _setupDisks(self) -> dict:
        r = {}
        for name in config.HEAD_MOTOR_NAME:
            r[name] = getattr(self._reachyHead, name)
        return r

    def getDisksInOrder(self) -> list:
        r = []
        for name in config.HEAD_MOTOR_NAME:
            r.append(self._disks[name])
        
        return r

    # ─── Motion ────────────────────────────────────────────────────────────────

    def lookAt(self, degAngles: list, duration: float = 1) -> None:
        cm.MKprint(f"Looking at {degAngles} in {duration}s", self.CLASS_NAME, self.CLASS_COLOR)
        self._reachyHead.look_at(x=degAngles[0], y=degAngles[1], z=degAngles[2], duration=duration)

    # ─── Record / Play ─────────────────────────────────────────────────────────

    def forwardKinematic(self, distance=1.0) -> list:
    
        rotation = R.from_euler('xyz', [self._disks[config.DISK_MOTOR_ROLL_NAME].present_position, self._disks[config.DISK_MOTOR_PITCH_NAME].present_position, self._disks[config.DISK_MOTOR_YAW_NAME].present_position], degrees=True)
        
        direction = rotation.apply([1.0, 0.0, 0.0])
        
        direction = direction / np.linalg.norm(direction)
        
        point = distance * direction
        
        return list(point)

    def invertKinematic(self, x: float, y: float, z: float) -> list:

        target = np.array([x, y, z], dtype=float)

        norm = np.linalg.norm(target)
        if norm < 1e-8:
            return [0.0, 0.0, 0.0]

        direction = target / norm

        # yaw + pitch only (plus stable pour tête robot)
        yaw = np.arctan2(direction[1], direction[0])
        pitch = -np.arctan2(direction[2], np.sqrt(direction[0]**2 + direction[1]**2))

        roll = 0.0

        return list(np.rad2deg([roll, pitch, yaw]))

    def recordHead(self, recordDurationSeconds: float, samplingFrequencyHertz: float) -> TimeSeries:
        trajectories = []
        samplingTime = 1.0 / samplingFrequencyHertz
        start        = time.time()

        cm.MKprint(f"Recording head for {recordDurationSeconds}s at {samplingFrequencyHertz}Hz.", self.CLASS_NAME, self.CLASS_COLOR)

        while (time.time() - start) < recordDurationSeconds:
            trajectories.append({name: joint for name, joint in zip(config.TIME_SERIE_HEAD_VALUES_NAME, self.forwardKinematic())})
            time.sleep(samplingTime)

        cm.MKprint("Recording done for head.", self.CLASS_NAME, self.CLASS_COLOR)
        return TimeSeries(samplingFrequencyHertz, recordDurationSeconds, trajectories, [0, 0, 1])

    def playHeadRecord(self, record: TimeSeries, startDuration: float = 3.0) -> None:

        if not record.jointPosition:
            return

        samplingTime = 1.0 / record.samplingFrequency

        cm.MKprint(f"Playing head record at {record.samplingFrequency}Hz.", self.CLASS_NAME, self.CLASS_COLOR)

        first_frame = record.jointPosition[0]

        first_point = [
            first_frame[name]
            for name in config.TIME_SERIE_HEAD_VALUES_NAME
        ]
        if(startDuration > 0):
            self._reachyHead.look_at(x=first_point[0], y=first_point[1], z=first_point[2], duration=startDuration)

        for frame in record.jointPosition:

            point = [
                frame[name]
                for name in config.TIME_SERIE_HEAD_VALUES_NAME
            ]
            
            roll, pitch, yaw = self.invertKinematic(*point)

            self._disks["neck_pitch"].goal_position = pitch
            self._disks["neck_yaw"].goal_position = yaw
            self._disks["neck_roll"].goal_position = roll

            time.sleep(samplingTime)