from reachy_sdk import ReachySDK
from concurrent.futures import ThreadPoolExecutor
import time

from . import armController as ac
from . import headController as hc
from . import torsoController as to
from . import collisionManager as col
from . import consoleManager as cm
from .timeSeries import TimeSeries
from . import config


class ReachyController:

    CLASS_NAME  : str = "Reachy controller"
    CLASS_COLOR : str = cm.Color.BRIGHT_RED

    def __init__(self, reachy: ReachySDK):
        self.reachy = reachy

        self.armLeft  = ac.ReachyArm(self.reachy, config.ARM_LEFT_ID)
        self.armRight = ac.ReachyArm(self.reachy, config.ARM_RIGHT_ID)
        self.head     = hc.ReachyHead(self.reachy)
        self.torso    = to.ReachyTorso()

        self.collision = col.CollisionManager(self.armRight, self.armLeft, self.torso)
        self.armLeft.setCollisionManager(self.collision)
        self.armRight.setCollisionManager(self.collision)

    # ─── Power ─────────────────────────────────────────────────────────────────

    def turnOn(self) -> None:
        self.reachy.turn_on("reachy")

    def turnOffSmooth(self) -> None:
        self.reachy.turn_off_smoothly("reachy")

    def turnOnSafe(self) -> None:
        """Turn on without snapping — syncs goal positions to present positions."""
        self.reachy.turn_on("reachy")
        time.sleep(0.5)
        for joint in self.reachy.joints.values():
            joint.goal_position = joint.present_position
    '''
    # ─── Record ────────────────────────────────────────────────────────────────

    def record(self, recordDurationSeconds: float, samplingFrequencyHertz: float,
               recordArmLeft: bool = True, recordArmRight: bool = True, recordHead: bool = True) -> TimeSeries:
        """
        Record all parts in parallel for recordDurationSeconds seconds.
        RETURN TimeSeries
        """
        if not (recordArmLeft or recordArmRight or recordHead):
            raise ValueError("At least one part must be recorded.")

        cm.MKprint("|--- Start recording ---|", self.CLASS_NAME, self.CLASS_COLOR)

        with ThreadPoolExecutor(max_workers=3) as executor:
            futures = {}
            if recordArmLeft:
                futures["l"] = executor.submit(self.armLeft.recordArm,   recordDurationSeconds, samplingFrequencyHertz)
            if recordArmRight:
                futures["r"] = executor.submit(self.armRight.recordArm,  recordDurationSeconds, samplingFrequencyHertz)
            if recordHead:
                futures["h"] = executor.submit(self.head.recordHead,     recordDurationSeconds, samplingFrequencyHertz)

            results = {k: f.result() for k, f in futures.items()}

        merged = None
        for ts in results.values():
            merged = ts if merged is None else merged + ts

        cm.MKprint("|--- Stop recording ----|", self.CLASS_NAME, self.CLASS_COLOR)
        return merged
    '''
    
    # ─── Play ──────────────────────────────────────────────────────────────────

    def playRecord(self, records: TimeSeries, startDuration: float = 3.0) -> None:
        """
        Play a TimeSeries record on all parts in parallel.
        PARAMETER records TYPE TimeSeries, startDuration TYPE float
        """
        cm.MKprint("|--- Start playing -----|", self.CLASS_NAME, self.CLASS_COLOR)
        with ThreadPoolExecutor(max_workers=3) as executor:
            executor.submit(self.armLeft.playArmRecord,   records, startDuration)
            executor.submit(self.armRight.playArmRecord,  records, startDuration)
            executor.submit(self.head.playHeadRecord,     records, startDuration)
        cm.MKprint("|--- Stop playing ------|", self.CLASS_NAME, self.CLASS_COLOR)
    