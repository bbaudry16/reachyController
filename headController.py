from reachy_sdk import trajectory, ReachySDK
import time

import config
import reachyPart as rp
import consoleManager as cm
from timeSeries import TimeSeries


class ReachyDisk:
    def __init__(self, maxAngleEuler: float, minAngleEuler: float):
        self.maxAngle = maxAngleEuler
        self.minAngle = minAngleEuler


class ReachyHead(rp.ReachyPart):

    DISK_NECK_ROLL  : ReachyDisk = ReachyDisk(60,  -60)
    DISK_NECK_PITCH : ReachyDisk = ReachyDisk(60,  -60)
    DISK_NECK_YAW   : ReachyDisk = ReachyDisk(360,   0)

    CLASS_NAME  : str = "Reachy head"
    CLASS_COLOR : str = cm.Color.YELLOW

    def __init__(self, reachy: ReachySDK):
        self._reachyHead = reachy.head
        self._disks      = self._setupDisks()

    def _setupDisks(self) -> dict:
        r = {}
        for name in config.HEAD_MOTOR_NAME:
            r[name] = getattr(self._reachyHead, name)
        return r

    # ─── Motion ────────────────────────────────────────────────────────────────

    def lookAt(self, degAngles: list, duration: float = 1) -> None:
        self._reachyHead.look_at(x=degAngles[0], y=degAngles[1], z=degAngles[2], duration=duration)

    # ─── Record / Play ─────────────────────────────────────────────────────────

    def recordHead(self, recordDurationSeconds: float, samplingFrequencyHertz: float) -> TimeSeries:
        trajectories = []
        samplingTime = 1.0 / samplingFrequencyHertz
        start        = time.time()

        cm.MKprint(f"Recording head for {recordDurationSeconds}s at {samplingFrequencyHertz}Hz.", self.CLASS_NAME, self.CLASS_COLOR)

        while (time.time() - start) < recordDurationSeconds:
            trajectories.append({name: joint.present_position for name, joint in self._disks.items()})
            time.sleep(samplingTime)

        cm.MKprint("Recording done for head.", self.CLASS_NAME, self.CLASS_COLOR)
        return TimeSeries(samplingFrequencyHertz, recordDurationSeconds, trajectories)

    def playHeadRecord(self, record: TimeSeries, startDuration: float = 3.0) -> None:
        firstPoint = {
            self._disks[m]: pos
            for m, pos in record.jointPosition[0].items()
            if m in self._disks
        }
        samplingTime = 1.0 / record.samplingFrequency

        cm.MKprint(f"Playing head record at {record.samplingFrequency}Hz.", self.CLASS_NAME, self.CLASS_COLOR)

        trajectory.goto(firstPoint, duration=startDuration)

        for jointsPositions in record.jointPosition:
            for name, pos in jointsPositions.items():
                if name in self._disks:
                    self._disks[name].goal_position = pos
            time.sleep(samplingTime)
