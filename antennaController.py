from reachy_sdk import trajectory, ReachySDK
import time

from . import config
from . import reachyPart as rp
from . import consoleManager as cm

class ReachyAntenna(rp.ReachyPart):
    CLASS_NAME  : str = "Reachy antenna"
    CLASS_COLOR : str = cm.Color.BRIGHT_GREEN

    def __init__(self, reachy: ReachySDK, antennaId : str):
        self._reachyHead = reachy.head
        self._antennaId : str = antennaId
        self._antenna = self._setupDisks()

    def _setupDisks(self):
        return getattr(self._reachyHead, self.sided(config.ANTENNA_MOTOR_NAME))
    
    def sided(self, motorName :str) -> str:
        return self._antennaId + "_" + motorName
    
    def setAntenna(self, angle: float, duration: float = 0.5) -> None:
        cm.MKprint(f"Antenna {self.sided('antenna')} going to angles {angle}° in {duration}s", self.CLASS_NAME, self.CLASS_COLOR)
                
        interpolated = {
                        self._antenna : angle
                    }

        trajectory.goto(interpolated, duration=duration)

    def vibrateAntenna(self, amplitude: float = 15.0, cycles: int = 3, speed: float = 0.08) -> None:
        
        base = self._antenna.present_position
        cm.MKprint(f"Vibrating antenna {self.sided('antenna')} x{cycles}", self.CLASS_NAME, self.CLASS_COLOR)
        for _ in range(cycles):
            
            interpolated = {
                        self._antenna : base + amplitude
                    }

            trajectory.goto(interpolated, duration=speed)

            interpolated = {
                        self._antenna : base - amplitude
                    }

            trajectory.goto(interpolated, duration=speed)
        
    
        interpolated = {
                    self._antenna : base
                }

        trajectory.goto(interpolated, duration=speed)