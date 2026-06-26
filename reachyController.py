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
    """
    Top-level controller that aggregates all Reachy subsystems.

    Provides unified access to arms, head, fans, and collision management,
    as well as coordinated record and replay across all parts.

    @cvar CLASS_NAME: Display name used in console output.
    @cvar CLASS_COLOR: Console color used for this class.
    @ivar reachy: Connected Reachy SDK instance.
    @ivar armLeft: Left arm controller.
    @ivar armRight: Right arm controller.
    @ivar head: Head controller.
    @ivar torso: Torso collision part.
    @ivar fans: Fan controller.
    @ivar collision: Collision manager shared by both arms.
    """

    CLASS_NAME  : str = "Reachy controller"
    CLASS_COLOR : str = cm.Color.MAGENTA

    @classmethod
    def instanciate(cls, ip: str = "localhost") -> "ReachyController":
        """
        Connect to Reachy at the given IP and return a ReachyController.

        @param ip: IP address of the Reachy robot.
        @type ip: str
        @rtype: ReachyController
        """
        return ReachyController(ReachySDK(host=ip))

    def __init__(self, reachy: ReachySDK):
        """
        @param reachy: Connected Reachy SDK instance.
        @type reachy: ReachySDK
        """
        self.reachy = reachy

        cm.MKprint("setup, ignore any collision warning except if it's not set at the end of this indentation ┐", self.CLASS_NAME, self.CLASS_COLOR)
        cm.addIndentation()

        self.armLeft  = ac.ReachyArm(self.reachy, config.ARM_LEFT_ID)
        self.armRight = ac.ReachyArm(self.reachy, config.ARM_RIGHT_ID)
        self.head     = hc.ReachyHead(self.reachy)
        self.torso    = to.ReachyTorso()
        self.fans     = fc.ReachyFan(self.reachy)

        self.collision = col.CollisionManager(self.armRight, self.armLeft, self.torso)
        self.armLeft.setCollisionManager(self.collision)
        self.armRight.setCollisionManager(self.collision)

        cm.removeIndentation()
        cm.MKprint("all setup correctly ┘", self.CLASS_NAME, self.CLASS_COLOR)

    def turnOn(self) -> None:
        """Turn on all Reachy motors."""
        self.reachy.turn_on("reachy")

    def turnOffSmooth(self) -> None:
        """Turn off all motors smoothly and switch fans off."""
        self.reachy.turn_off_smoothly("reachy")
        self.fans.turnOffAll()

    def turnOnSafe(self) -> None:
        """
        Turn on motors without snapping by syncing goal positions to present positions.
        """
        self.reachy.turn_on("reachy")
        time.sleep(0.5)
        for joint in self.reachy.joints.values():
            joint.goal_position = joint.present_position

    def record(self, recordDurationSeconds: float, samplingFrequencyHertz: float,
               recordArmLeft: bool = True, recordArmRight: bool = True,
               recordHead: bool = True) -> TimeSeries:
        """
        Record all selected parts in parallel.

        @param recordDurationSeconds: Recording duration in seconds.
        @type recordDurationSeconds: float
        @param samplingFrequencyHertz: Sampling frequency in Hz.
        @type samplingFrequencyHertz: float
        @param recordArmLeft: Include the left arm.
        @type recordArmLeft: bool
        @param recordArmRight: Include the right arm.
        @type recordArmRight: bool
        @param recordHead: Include the head.
        @type recordHead: bool
        @rtype: TimeSeries
        @raise ValueError: If no parts are selected.
        """
        if not (recordArmLeft or recordArmRight or recordHead):
            raise ValueError("At least one part must be recorded.")

        cm.MKprint("start recording ┐", self.CLASS_NAME, self.CLASS_COLOR)
        cm.addIndentation()

        with ThreadPoolExecutor(max_workers=3) as executor:
            futures      = {}
            partInvolved = []
            if recordHead:
                partInvolved.append("h")
                futures["h"] = executor.submit(self.head.recordHead,    recordDurationSeconds, samplingFrequencyHertz)
            if recordArmLeft:
                partInvolved.append("l")
                futures["l"] = executor.submit(self.armLeft.recordArm,  recordDurationSeconds, samplingFrequencyHertz)
            if recordArmRight:
                partInvolved.append("r")
                futures["r"] = executor.submit(self.armRight.recordArm, recordDurationSeconds, samplingFrequencyHertz)

            results = {k: futures[k].result() for k in partInvolved}

        merged = None
        for ts in results.values():
            merged = ts if merged is None else merged + ts

        cm.removeIndentation()
        cm.MKprint("stop recording ┘", self.CLASS_NAME, self.CLASS_COLOR)
        return merged

    def playRecord(self, records: TimeSeries, startDuration: float = 3.0) -> None:
        """
        Replay a TimeSeries on all parts in parallel.

        @param records: Time series to replay.
        @type records: TimeSeries
        @param startDuration: Duration to move to the first frame, in seconds.
        @type startDuration: float
        """
        cm.MKprint("start playing ┐", self.CLASS_NAME, self.CLASS_COLOR)
        cm.addIndentation()
        with ThreadPoolExecutor(max_workers=3) as executor:
            executor.submit(self.armLeft.playArmRecord,  records, startDuration)
            executor.submit(self.armRight.playArmRecord, records, startDuration)
            executor.submit(self.head.playHeadRecord,    records, startDuration)
        cm.removeIndentation()
        cm.MKprint("stop playing ┘", self.CLASS_NAME, self.CLASS_COLOR)
