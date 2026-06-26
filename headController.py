from reachy_sdk.camera import ZoomLevel
from reachy_sdk import trajectory, ReachySDK
import time
import numpy as np
from scipy.spatial.transform import Rotation as R

from . import config
from . import reachyPart as rp
from . import consoleManager as cm
from .antennaController import ReachyAntenna
from .timeSeries import TimeSeries


class ReachyDisk:
    """
    Angle limits for a single head disk motor.

    @ivar maxAngle: Maximum allowed angle in degrees.
    @ivar minAngle: Minimum allowed angle in degrees.
    """

    def __init__(self, maxAngleEuler: float, minAngleEuler: float):
        """
        @param maxAngleEuler: Maximum angle in degrees.
        @type maxAngleEuler: float
        @param minAngleEuler: Minimum angle in degrees.
        @type minAngleEuler: float
        """
        self.maxAngle = maxAngleEuler
        self.minAngle = minAngleEuler


class ReachyHead(rp.ReachyPart):
    """
    Controller for the Reachy head, including neck disks, antennas, and cameras.

    @cvar DISK_NECK_ROLL: Roll axis limits.
    @cvar DISK_NECK_PITCH: Pitch axis limits.
    @cvar DISK_NECK_YAW: Yaw axis limits.
    @cvar CLASS_NAME: Display name used in console output.
    @cvar CLASS_COLOR: Console color used for this class.
    @ivar _reachyHead: SDK head object.
    @ivar _disks: Mapping of motor name to SDK motor object.
    @ivar antennaLeft: Left antenna controller.
    @ivar antennaRight: Right antenna controller.
    @ivar cameraLeft: Left camera from the SDK.
    @ivar cameraRight: Right camera from the SDK.
    """

    DISK_NECK_ROLL  : ReachyDisk = ReachyDisk(60,  -60)
    DISK_NECK_PITCH : ReachyDisk = ReachyDisk(60,  -60)
    DISK_NECK_YAW   : ReachyDisk = ReachyDisk(360,   0)

    CLASS_NAME  : str = "Reachy head"
    CLASS_COLOR : str = cm.Color.BRIGHT_BLUE

    def __init__(self, reachy: ReachySDK):
        """
        @param reachy: Connected Reachy SDK instance.
        @type reachy: ReachySDK
        """
        self._reachyHead  = reachy.head
        self._disks       = self._setupDisks()
        self.antennaLeft  = ReachyAntenna(reachy, config.ARM_LEFT_ID)
        self.antennaRight = ReachyAntenna(reachy, config.ARM_RIGHT_ID)
        self.cameraLeft   = reachy.left_camera
        self.cameraRight  = reachy.right_camera

    def _setupDisks(self) -> dict:
        """
        Build the disk motor dictionary from the SDK head object.

        @rtype: dict
        """
        return {name: getattr(self._reachyHead, name) for name in config.HEAD_MOTOR_NAME}

    def getDisksInOrder(self) -> list:
        """
        Return the disk motor objects in the canonical order defined by config.

        @rtype: list
        """
        return [self._disks[name] for name in config.HEAD_MOTOR_NAME]

    def getHeadAngles(self) -> list:
        """
        Return the current neck angles as [roll, pitch, yaw] in degrees.

        @rtype: list[float]
        """
        return [round(self._disks[name].present_position, 2) for name in config.HEAD_MOTOR_NAME]

    def gotoHeadAngles(self, angles: list, duration: float = None) -> None:
        """
        Move the neck to target [roll, pitch, yaw] angles.

        Duration is auto-computed from the maximum angular delta if not provided.

        @param angles: Target angles in degrees as [roll, pitch, yaw].
        @type angles: list[float]
        @param duration: Movement duration in seconds, or None for auto.
        @type duration: float or None
        """
        current   = self.getHeadAngles()
        max_delta = max(abs(a - c) for a, c in zip(angles, current))

        if duration is None:
            duration = max(0.5, min(2.0, max_delta / 15.0 * 0.5))

        cm.MKprint(
            f"Head goto {angles} delta={max_delta:.1f}° in {duration:.2f}s",
            self.CLASS_NAME, self.CLASS_COLOR
        )
        trajectory.goto({
            self._disks[config.DISK_MOTOR_ROLL_NAME]:  angles[0],
            self._disks[config.DISK_MOTOR_PITCH_NAME]: angles[1],
            self._disks[config.DISK_MOTOR_YAW_NAME]:   angles[2],
        }, duration=duration)

    def lookAt(self, degAngles: list, duration: float = 1) -> None:
        """
        Point the head at a 3D position using the SDK look_at interface.

        @param degAngles: Target [x, y, z] position in the robot frame.
        @type degAngles: list[float]
        @param duration: Movement duration in seconds.
        @type duration: float
        """
        cm.MKprint(f"Looking at {degAngles} in {duration}s", self.CLASS_NAME, self.CLASS_COLOR)
        self._reachyHead.look_at(x=degAngles[0], y=degAngles[1], z=degAngles[2], duration=duration)

    def forwardKinematic(self, distance: float = 1.0) -> list:
        """
        Return the 3D point the head is looking at, at the given distance.

        @param distance: Distance in meters from the head origin.
        @type distance: float
        @rtype: list[float]
        """
        rotation  = R.from_euler(
            'xyz',
            [self._disks[config.DISK_MOTOR_ROLL_NAME].present_position,
             self._disks[config.DISK_MOTOR_PITCH_NAME].present_position,
             self._disks[config.DISK_MOTOR_YAW_NAME].present_position],
            degrees=True
        )
        direction = rotation.apply([1.0, 0.0, 0.0])
        direction = direction / np.linalg.norm(direction)
        return list(distance * direction)

    def invertKinematic(self, x: float, y: float, z: float) -> list:
        """
        Compute neck angles (roll, pitch, yaw) to look at the given 3D point.

        @param x: X coordinate of the target.
        @type x: float
        @param y: Y coordinate of the target.
        @type y: float
        @param z: Z coordinate of the target.
        @type z: float
        @rtype: list[float]
        @return: [roll, pitch, yaw] in degrees.
        """
        target = np.array([x, y, z], dtype=float)
        norm   = np.linalg.norm(target)
        if norm < 1e-8:
            return [0.0, 0.0, 0.0]
        direction = target / norm
        yaw   = np.arctan2(direction[1], direction[0])
        pitch = -np.arctan2(direction[2], np.sqrt(direction[0]**2 + direction[1]**2))
        roll  = 0.0
        return list(np.rad2deg([roll, pitch, yaw]))

    def recordHead(self, recordDurationSeconds: float, samplingFrequencyHertz: float) -> TimeSeries:
        """
        Record head orientation as a look-at point time series.

        @param recordDurationSeconds: Recording duration in seconds.
        @type recordDurationSeconds: float
        @param samplingFrequencyHertz: Sampling frequency in Hz.
        @type samplingFrequencyHertz: float
        @rtype: TimeSeries
        """
        trajectories = []
        samplingTime = 1.0 / samplingFrequencyHertz
        start        = time.time()

        cm.MKprint(
            f"Recording head for {recordDurationSeconds}s at {samplingFrequencyHertz}Hz.",
            self.CLASS_NAME, self.CLASS_COLOR
        )

        while (time.time() - start) < recordDurationSeconds:
            trajectories.append({
                name: joint
                for name, joint in zip(config.TIME_SERIE_HEAD_VALUES_NAME, self.forwardKinematic())
            })
            time.sleep(samplingTime)

        cm.MKprint("Recording done for head.", self.CLASS_NAME, self.CLASS_COLOR)
        return TimeSeries(samplingFrequencyHertz, recordDurationSeconds, trajectories, [0, 0, 1])

    def playHeadRecord(self, record: TimeSeries, startDuration: float = 3.0) -> None:
        """
        Replay a recorded head time series.

        @param record: Time series containing look-at point frames.
        @type record: TimeSeries
        @param startDuration: Duration to move to the first frame, in seconds.
        @type startDuration: float
        """
        if not record.jointPosition:
            return

        samplingTime = 1.0 / record.samplingFrequency
        cm.MKprint(
            f"Playing head record at {record.samplingFrequency}Hz.",
            self.CLASS_NAME, self.CLASS_COLOR
        )

        first_frame = record.jointPosition[0]
        first_point = [first_frame[name] for name in config.TIME_SERIE_HEAD_VALUES_NAME]

        if startDuration > 0:
            self._reachyHead.look_at(
                x=first_point[0], y=first_point[1], z=first_point[2],
                duration=startDuration
            )

        for frame in record.jointPosition:
            point        = [frame[name] for name in config.TIME_SERIE_HEAD_VALUES_NAME]
            roll, pitch, yaw = self.invertKinematic(*point)
            self._disks["neck_pitch"].goal_position = pitch
            self._disks["neck_yaw"].goal_position   = yaw
            self._disks["neck_roll"].goal_position  = roll
            time.sleep(samplingTime)

    def setCameraZoomLevel(self, zoomLevel: "ZoomLevel") -> None:
        """
        Set the zoom level on both cameras and trigger autofocus.

        @param zoomLevel: Zoom level constant from the SDK.
        @type zoomLevel: ZoomLevel
        """
        self.cameraLeft.zoom_level  = zoomLevel
        self.cameraRight.zoom_level = zoomLevel
        self.cameraLeft.start_autofocus()
        self.cameraRight.start_autofocus()
