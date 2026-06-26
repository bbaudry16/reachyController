from reachy_sdk import trajectory, ReachySDK
import time

from . import config
from . import reachyPart as rp
from . import consoleManager as cm


class ReachyAntenna(rp.ReachyPart):
    """
    Controller for a single Reachy antenna motor.

    @cvar CLASS_NAME: Display name used in console output.
    @cvar CLASS_COLOR: Console color used for this class.
    @ivar _reachyHead: Reference to the SDK head object.
    @ivar _antennaId: Side identifier ('l' or 'r').
    @ivar _antenna: SDK motor object for this antenna.
    """

    CLASS_NAME  : str = "Reachy antenna"
    CLASS_COLOR : str = cm.Color.BRIGHT_GREEN

    def __init__(self, reachy: ReachySDK, antennaId: str):
        """
        @param reachy: Connected Reachy SDK instance.
        @type reachy: ReachySDK
        @param antennaId: Side identifier, 'l' or 'r'.
        @type antennaId: str
        """
        self._reachyHead = reachy.head
        self._antennaId  = antennaId
        self._antenna    = self._setupAntenna()

    def _setupAntenna(self):
        """
        Retrieve the SDK motor object for this antenna.

        @return: SDK antenna motor.
        """
        return getattr(self._reachyHead, self.sided(config.ANTENNA_MOTOR_NAME))

    def sided(self, motorName: str) -> str:
        """
        Prefix a motor name with the antenna side identifier.

        @param motorName: Base motor name.
        @type motorName: str
        @rtype: str
        """
        return self._antennaId + "_" + motorName

    def setAntenna(self, angle: float, duration: float = 0.5) -> None:
        """
        Move the antenna to an absolute angle.

        @param angle: Target angle in degrees.
        @type angle: float
        @param duration: Movement duration in seconds.
        @type duration: float
        """
        cm.MKprint(
            f"Antenna {self.sided('antenna')} going to {angle}° in {duration}s",
            self.CLASS_NAME, self.CLASS_COLOR
        )
        trajectory.goto({self._antenna: angle}, duration=duration)

    def vibrateAntenna(self, amplitude: float = 15.0, cycles: int = 3, speed: float = 0.08) -> None:
        """
        Oscillate the antenna back and forth around its current position.

        @param amplitude: Half-swing amplitude in degrees.
        @type amplitude: float
        @param cycles: Number of full oscillation cycles.
        @type cycles: int
        @param speed: Duration of each half-swing in seconds.
        @type speed: float
        """
        base = self._antenna.present_position
        cm.MKprint(
            f"Vibrating antenna {self.sided('antenna')} x{cycles}",
            self.CLASS_NAME, self.CLASS_COLOR
        )
        for _ in range(cycles):
            trajectory.goto({self._antenna: base + amplitude}, duration=speed)
            trajectory.goto({self._antenna: base - amplitude}, duration=speed)
        trajectory.goto({self._antenna: base}, duration=speed)

    def doABarelRoll(self, cycles: int = 1, speed: float = 0.08) -> None:
        """
        Sweep the antenna across its full physical range and return to base.

        @param cycles: Number of full sweeps.
        @type cycles: int
        @param speed: Duration in seconds for the initial movement.
        @type speed: float
        """
        base = self._antenna.present_position
        cm.MKprint(
            f"Rotating antenna {self.sided('antenna')} x{cycles}",
            self.CLASS_NAME, self.CLASS_COLOR
        )
        MAX =  149.0
        MIN = -149.0
        for _ in range(cycles):
            trajectory.goto({self._antenna: MAX}, duration=speed)
            trajectory.goto({self._antenna: MIN}, duration=speed * 2)
            trajectory.goto({self._antenna: MAX}, duration=speed * 2)
        trajectory.goto({self._antenna: base}, duration=speed)
