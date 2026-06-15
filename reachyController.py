from reachy_sdk import ReachySDK
from concurrent.futures import ThreadPoolExecutor
import time

from . import armController as ac
from . import headController as hc
from . import torsoController as to
from . import collisionManager as col
from . import consoleManager as cm
from . import fanController as fc
from .timeSeries import TimeSeries
from . import config


class ReachyController:

    CLASS_NAME  : str = "Reachy controller"
    CLASS_COLOR : str = cm.Color.MAGENTA

    @classmethod
    def instanciate(self, ip : str = "localhost"):
        reachy  = ReachySDK(host=ip)
        return ReachyController(reachy)

    def __init__(self, reachy: ReachySDK):
        self.reachy = reachy

        cm.MKprint("setup, ignore any collision warning except if it's not set at the end of this intentation ┐", self.CLASS_NAME, self.CLASS_COLOR)
        cm.addIntentation()
        self.armLeft  = ac.ReachyArm(self.reachy, config.ARM_LEFT_ID)
        self.armRight = ac.ReachyArm(self.reachy, config.ARM_RIGHT_ID)
        self.head     = hc.ReachyHead(self.reachy)
        self.torso    = to.ReachyTorso()

        self.fans = fc.ReachyFan(self.reachy)

        self.collision = col.CollisionManager(self.armRight, self.armLeft, self.torso)
        self.armLeft.setCollisionManager(self.collision)
        self.armRight.setCollisionManager(self.collision)
        cm.removeIntentation()
        cm.MKprint("all setup correctly ┘", self.CLASS_NAME, self.CLASS_COLOR)

    def turnOn(self) -> None:
        self.reachy.turn_on("reachy")

    def turnOffSmooth(self) -> None:
        self.reachy.turn_off_smoothly("reachy")
        self.fans.turnOffAll()

    def turnOnSafe(self) -> None:
        """Turn on without snapping — syncs goal positions to present positions."""
        self.reachy.turn_on("reachy")
        time.sleep(0.5)
        for joint in self.reachy.joints.values():
            joint.goal_position = joint.present_position
    

    def record(self, recordDurationSeconds: float, samplingFrequencyHertz: float,
               recordArmLeft: bool = True, recordArmRight: bool = True, recordHead: bool = True) -> TimeSeries:
        """
        Record all parts in parallel for recordDurationSeconds seconds.
        RETURN TimeSeries
        """
        if not (recordArmLeft or recordArmRight or recordHead):
            raise ValueError("At least one part must be recorded.")

        cm.MKprint("start recording ┐", self.CLASS_NAME, self.CLASS_COLOR)
        cm.addIntentation()

        with ThreadPoolExecutor(max_workers=3) as executor:
            futures = {}
            partInvolved = []
            if recordHead:
                partInvolved.append("h")
                futures["h"] = executor.submit(self.head.recordHead,     recordDurationSeconds, samplingFrequencyHertz)
            if recordArmLeft:
                partInvolved.append("l")
                futures["l"] = executor.submit(self.armLeft.recordArm,   recordDurationSeconds, samplingFrequencyHertz)
            if recordArmRight:
                partInvolved.append("r")
                futures["r"] = executor.submit(self.armRight.recordArm,  recordDurationSeconds, samplingFrequencyHertz)
            

            results = {k: futures[k].result() for k in partInvolved}

        merged = None
        for ts in results.values():
            merged = ts if merged is None else merged + ts

        cm.removeIntentation()
        cm.MKprint("stop recording ┘", self.CLASS_NAME, self.CLASS_COLOR)
        
        return merged
    
    
    # ─── Play ──────────────────────────────────────────────────────────────────

    def playRecord(self, records: TimeSeries, startDuration: float = 3.0) -> None:
        """
        Play a TimeSeries record on all parts in parallel.
        PARAMETER records TYPE TimeSeries, startDuration TYPE float
        """
        cm.MKprint("start playing ┐", self.CLASS_NAME, self.CLASS_COLOR)
        cm.addIntentation()
        with ThreadPoolExecutor(max_workers=3) as executor:
            executor.submit(self.armLeft.playArmRecord,   records, startDuration)
            executor.submit(self.armRight.playArmRecord,  records, startDuration)
            executor.submit(self.head.playHeadRecord,     records, startDuration)
        cm.removeIntentation()
        cm.MKprint("stop playing ┘", self.CLASS_NAME, self.CLASS_COLOR)
        
    