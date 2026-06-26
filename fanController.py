from reachy_sdk import ReachySDK

from . import config
from . import reachyPart as rp
from . import consoleManager as cm


class FanMode:
    """Constants for fan operating modes."""
    OFF : str = "off"
    ON  : str = "on"


class ReachyFan(rp.ReachyPart):
    """
    Controller for all Reachy cooling fans with optional temperature-based automation.

    @cvar CLASS_NAME: Display name used in console output.
    @cvar CLASS_COLOR: Console color used for this class.
    @ivar _reachy: Connected Reachy SDK instance.
    @ivar _fans: Mapping of fan name to SDK fan object.
    @ivar _mode: Current fan mode (on/off).
    @ivar _auto: Whether temperature-based auto control is active.
    """

    CLASS_NAME  : str = "Reachy fan"
    CLASS_COLOR : str = cm.Color.BRIGHT_CYAN

    def __init__(self, reachy: ReachySDK):
        """
        @param reachy: Connected Reachy SDK instance.
        @type reachy: ReachySDK
        """
        self._reachy = reachy
        self._fans   = self._setupFans()
        self._mode   : str  = FanMode.OFF
        self._auto   : bool = True

    def _setupFans(self) -> dict:
        """
        Discover and register all available fans from the SDK.

        @rtype: dict
        @return: Mapping of fan name to SDK fan object.
        """
        fans   = {}
        holder = self._reachy.fans
        raw    = getattr(holder, "_fans", None)

        if raw is not None and isinstance(raw, dict):
            source = raw
        else:
            source = {k: v for k, v in vars(holder).items() if not k.startswith("_")}

        for name, fan in source.items():
            fans[name] = fan
            cm.MKprint(f"Fan registered: {name}", self.CLASS_NAME, self.CLASS_COLOR)

        if not fans:
            cm.MKprintWarning(
                "No fans found in reachy.fans — check SDK version.",
                self.CLASS_NAME, self.CLASS_COLOR
            )
        return fans

    def setMode(self, mode: str) -> None:
        """
        Set all fans to the given mode.

        @param mode: Target mode, either L{FanMode.ON} or L{FanMode.OFF}.
        @type mode: str
        """
        if mode not in (FanMode.OFF, FanMode.ON):
            cm.MKprintWarning(
                f"Unknown fan mode '{mode}', expected 'off' or 'on'.",
                self.CLASS_NAME, self.CLASS_COLOR
            )
            return

        self._mode = mode
        cm.MKprint(f"Setting all fans to: {mode}", self.CLASS_NAME, self.CLASS_COLOR)

        for name, fan in self._fans.items():
            try:
                fan.off() if mode == FanMode.OFF else fan.on()
                cm.MKprint(f"  {name} -> {mode}", self.CLASS_NAME, self.CLASS_COLOR)
            except Exception as e:
                cm.MKprintWarning(f"  {name} failed: {e}", self.CLASS_NAME, self.CLASS_COLOR)

    def setFan(self, fanName: str, mode: str) -> None:
        """
        Set a single fan by name to the given mode.

        @param fanName: Name of the fan to control.
        @type fanName: str
        @param mode: Target mode, either L{FanMode.ON} or L{FanMode.OFF}.
        @type mode: str
        """
        if fanName not in self._fans:
            cm.MKprintWarning(
                f"Unknown fan '{fanName}'. Available: {list(self._fans.keys())}",
                self.CLASS_NAME, self.CLASS_COLOR
            )
            return

        fan = self._fans[fanName]
        try:
            fan.off() if mode == FanMode.OFF else fan.on()
            cm.MKprint(f"Fan '{fanName}' -> {mode}", self.CLASS_NAME, self.CLASS_COLOR)
        except Exception as e:
            cm.MKprintWarning(f"Fan '{fanName}' failed: {e}", self.CLASS_NAME, self.CLASS_COLOR)

    def _collectTemperatures(self) -> dict:
        """
        Collect temperatures from all joints and antenna motors.

        @rtype: dict
        @return: Mapping of joint name to temperature in Celsius.
        """
        temperatures = {}

        for name, joint in self._reachy.joints.items():
            temp = getattr(joint, "temperature", None)
            if temp is not None:
                temperatures[name] = temp

        head = getattr(self._reachy, "head", None)
        if head is not None:
            for attr in ("l_antenna", "r_antenna"):
                joint = getattr(head, attr, None)
                if joint is not None:
                    temp = getattr(joint, "temperature", None)
                    if temp is not None:
                        temperatures[attr] = temp

        return temperatures

    def updateFromTemperature(self, temperatures: dict, threshold: float = config.FAN_THRESHOLD) -> None:
        """
        Switch fans on or off based on the maximum observed temperature.

        @param temperatures: Mapping of motor name to temperature in Celsius.
        @type temperatures: dict
        @param threshold: Temperature threshold for turning fans on.
        @type threshold: float
        """
        if not temperatures:
            cm.MKprintWarning("No temperature data provided.", self.CLASS_NAME, self.CLASS_COLOR)
            return

        valid = {k: v for k, v in temperatures.items() if v is not None}
        if not valid:
            cm.MKprintWarning("All temperature values are None.", self.CLASS_NAME, self.CLASS_COLOR)
            return

        maxTemp  = max(valid.values())
        maxMotor = max(valid, key=valid.get)

        cm.MKprint(f"Max temperature: {maxTemp:.1f}°C ({maxMotor})", self.CLASS_NAME, self.CLASS_COLOR)

        targetMode = FanMode.ON if maxTemp >= threshold else FanMode.OFF

        if targetMode != self._mode:
            cm.MKprintSafety(
                f"Temperature {maxTemp:.1f}°C -> switching fans '{self._mode}' -> '{targetMode}'",
                self.CLASS_NAME, self.CLASS_COLOR
            )
            self.setMode(targetMode)

    def enableAuto(self) -> None:
        """Enable temperature-based automatic fan control."""
        self._auto = True
        cm.MKprint("Auto fan control enabled.", self.CLASS_NAME, self.CLASS_COLOR)

    def disableAuto(self) -> None:
        """Disable temperature-based automatic fan control."""
        self._auto = False
        cm.MKprint("Auto fan control disabled.", self.CLASS_NAME, self.CLASS_COLOR)

    def tick(self) -> None:
        """
        Run one auto-control cycle.

        Reads temperatures and updates fan state if auto mode is enabled.
        Call this periodically from a control loop.
        """
        if not self._auto:
            return
        try:
            self.updateFromTemperature(self._collectTemperatures())
        except Exception as e:
            cm.MKprintWarning(f"Failed to read temperatures: {e}", self.CLASS_NAME, self.CLASS_COLOR)

    def getMode(self) -> str:
        """
        Return the current fan mode.

        @rtype: str
        """
        return self._mode

    def getFanNames(self) -> list:
        """
        Return the names of all registered fans.

        @rtype: list
        """
        return list(self._fans.keys())

    def turnOffAll(self) -> None:
        """Turn all fans off."""
        self.setMode(FanMode.OFF)

    def turnOnAll(self) -> None:
        """Turn all fans on."""
        self.setMode(FanMode.ON)

    def _getTemperaturesForFan(self, fanName: str) -> list[tuple[str, float]]:
        """
        Return temperature readings for the joints associated with a fan.

        @param fanName: Fan name (expected to share a prefix with joint names).
        @type fanName: str
        @rtype: list[tuple[str, float]]
        """
        prefix  = fanName.replace("_fan", "")
        results = []

        for jointName, joint in self._reachy.joints.items():
            if jointName.startswith(prefix):
                temp = getattr(joint, "temperature", None)
                results.append((jointName, temp))

        if not results:
            head = getattr(self._reachy, "head", None)
            if head is not None:
                joint = getattr(head, prefix, None)
                if joint is not None:
                    temp = getattr(joint, "temperature", None)
                    results.append((prefix, temp))

        return results

    def printState(self) -> None:
        """Print the current state and temperatures of all fans to the console."""
        cm.MKprint("Fan states:", self.CLASS_NAME, self.CLASS_COLOR)
        for fan in self._fans.values():
            state = "ON " if fan.is_on else "OFF"
            temps = self._getTemperaturesForFan(fan.name)

            if temps:
                tempStr = "  ".join(
                    f"{name}: {f'{temp:.1f}°C' if temp is not None else 'N/A'}"
                    for name, temp in temps
                )
                cm.MKprint(f"  [{state}] {fan.name:<20} {tempStr}", self.CLASS_NAME, self.CLASS_COLOR)
            else:
                cm.MKprint(f"  [{state}] {fan.name}", self.CLASS_NAME, self.CLASS_COLOR)
