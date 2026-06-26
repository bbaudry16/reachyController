import json
import csv
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

from . import config
from . import consoleManager as cm
from .DBA_multivariate import performDBA


CLASS_NAME  : str = "Time serie"
CLASS_COLOR : str = cm.Color.BRIGHT_GREEN


class TimeSeries:
    """
    Immutable-style container for multi-joint robot motion recordings.

    Stores per-frame joint positions for the right arm, left arm, and head,
    along with the sampling frequency and total duration. Supports arithmetic
    operators for merging, concatenating, slicing, repeating, and averaging
    multiple series via DTW Barycenter Averaging (DBA).

    @cvar rightJoint: Sided joint names for the right arm.
    @cvar leftJoint: Sided joint names for the left arm.
    @cvar headJoint: Joint names for the head look-at point.
    @cvar jointLabel: Full CSV column header list.
    @ivar samplingFrequency: Sampling frequency in Hz.
    @ivar recordDuration: Total duration in seconds.
    @ivar jointPosition: List of frame dicts mapping joint name to angle.
    @ivar flags: [right_active, left_active, head_active] boolean list.
    """

    rightJoint : list = [str(config.ARM_RIGHT_ID) + "_" + x for x in config.ARM_MOTOR_NAME]
    leftJoint  : list = [str(config.ARM_LEFT_ID)  + "_" + x for x in config.ARM_MOTOR_NAME]
    headJoint  : list = ["head_x", "head_y", "head_z"]
    jointLabel : list = ["frame", "timestamp"] + rightJoint + leftJoint + headJoint

    def __init__(self, samplingFrequency: float, recordDurationSeconds: float,
                 jointPosition: list = None, flags: list = None):
        """
        @param samplingFrequency: Sampling frequency in Hz.
        @type samplingFrequency: float
        @param recordDurationSeconds: Total recording duration in seconds.
        @type recordDurationSeconds: float
        @param jointPosition: List of frame dicts. Defaults to empty list.
        @type jointPosition: list or None
        @param flags: [right_active, left_active, head_active]. Defaults to [True, True, True].
        @type flags: list or None
        """
        self.samplingFrequency = samplingFrequency
        self.recordDuration    = recordDurationSeconds
        self.jointPosition     = jointPosition if jointPosition is not None else []
        self.flags             = (flags if flags is not None else [True, True, True]).copy()

    def mergeLists(self, a: list, b: list, aFlags: list, bFlags: list) -> tuple:
        """
        Merge two frame lists by overlaying b's active channels onto a.

        Shorter lists are padded with their last frame.

        @param a: Base frame list.
        @type a: list
        @param b: Overlay frame list.
        @type b: list
        @param aFlags: Active channel flags for a.
        @type aFlags: list
        @param bFlags: Active channel flags for b.
        @type bFlags: list
        @rtype: tuple[list, list]
        @return: (merged frames, merged flags).
        """
        maxLen = max(len(a), len(b))
        if not a:
            a = [{}]
        if not b:
            b = [{}]
        aLast  = a[-1]
        bLast  = b[-1]
        merged = []

        for i in range(maxLen):
            frameA = a[i] if i < len(a) else aLast
            frameB = b[i] if i < len(b) else bLast
            frame  = frameA.copy()

            if bFlags[0]:
                for joint in self.rightJoint:
                    if joint in frameB:
                        frame[joint] = frameB[joint]
            if bFlags[1]:
                for joint in self.leftJoint:
                    if joint in frameB:
                        frame[joint] = frameB[joint]
            if bFlags[2]:
                for joint in self.headJoint:
                    if joint in frameB:
                        frame[joint] = frameB[joint]
            if "timestamp" in frameB:
                frame["timestamp"] = frameB["timestamp"]

            merged.append(frame)

        mergedFlags = [aFlags[i] or bFlags[i] for i in range(3)]
        return merged, mergedFlags

    def __add__(self, other: "TimeSeries") -> "TimeSeries":
        """
        Merge two same-duration time series by overlaying the other's active channels.

        @param other: Time series to merge with.
        @type other: TimeSeries
        @rtype: TimeSeries
        @raise ValueError: If sampling frequencies differ.
        """
        if self.samplingFrequency != other.samplingFrequency:
            raise ValueError("Cannot merge TimeSeries with different sampling frequencies.")
        duration       = max(self.recordDuration, other.recordDuration)
        frames, flags  = self.mergeLists(self.jointPosition, other.jointPosition, self.flags, other.flags)
        return TimeSeries(self.samplingFrequency, duration, frames, flags)

    def __rshift__(self, other: "TimeSeries") -> "TimeSeries":
        """
        Concatenate self followed by other.

        @param other: Time series to append.
        @type other: TimeSeries
        @rtype: TimeSeries
        @raise ValueError: If sampling frequencies differ.
        """
        if self.samplingFrequency != other.samplingFrequency:
            raise ValueError("Cannot concatenate TimeSeries with different sampling frequencies.")
        frames   = [f.copy() for f in self.jointPosition] + [f.copy() for f in other.jointPosition]
        duration = self.recordDuration + other.recordDuration
        flags    = [self.flags[i] or other.flags[i] for i in range(3)]
        return TimeSeries(self.samplingFrequency, duration, frames, flags)

    def __lshift__(self, other: "TimeSeries") -> "TimeSeries":
        """
        Concatenate other followed by self.

        @param other: Time series to prepend.
        @type other: TimeSeries
        @rtype: TimeSeries
        @raise ValueError: If sampling frequencies differ.
        """
        if self.samplingFrequency != other.samplingFrequency:
            raise ValueError("Cannot concatenate TimeSeries with different sampling frequencies.")
        frames   = [f.copy() for f in other.jointPosition] + [f.copy() for f in self.jointPosition]
        duration = other.recordDuration + self.recordDuration
        flags    = [self.flags[i] or other.flags[i] for i in range(3)]
        return TimeSeries(self.samplingFrequency, duration, frames, flags)

    def __getitem__(self, key) -> "TimeSeries":
        """
        Slice the time series by frame index.

        @param key: Slice object.
        @type key: slice
        @rtype: TimeSeries
        @raise TypeError: If key is not a slice.
        """
        if not isinstance(key, slice):
            raise TypeError("TimeSeries only supports slice indexing.")
        frames   = [f.copy() for f in self.jointPosition[key]]
        duration = len(frames) / self.samplingFrequency
        return TimeSeries(self.samplingFrequency, duration, frames, self.flags.copy())

    def __mul__(self, n: int) -> "TimeSeries":
        """
        Repeat the time series n times.

        @param n: Repeat count (must be a positive integer).
        @type n: int
        @rtype: TimeSeries
        @raise ValueError: If n is not a positive integer.
        """
        if not isinstance(n, int) or n < 1:
            raise ValueError("Repeat count must be a positive integer.")
        frames   = [f.copy() for f in self.jointPosition] * n
        duration = self.recordDuration * n
        return TimeSeries(self.samplingFrequency, duration, frames, self.flags.copy())

    def __rmul__(self, n: int) -> "TimeSeries":
        """
        Right-multiply: same as L{__mul__}.

        @param n: Repeat count.
        @type n: int
        @rtype: TimeSeries
        """
        return self.__mul__(n)

    def __len__(self) -> int:
        """
        Return the number of frames.

        @rtype: int
        """
        return len(self.jointPosition)

    def __or__(self, other: "TimeSeries") -> "TimeSeries":
        """
        Average this series with another using DBA.

        @param other: Time series to average with.
        @type other: TimeSeries
        @rtype: TimeSeries
        """
        return TimeSeries.dba([self, other])

    def reverse(self) -> "TimeSeries":
        """
        Return a time-reversed copy of this series.

        @rtype: TimeSeries
        """
        frames = [f.copy() for f in reversed(self.jointPosition)]
        return TimeSeries(self.samplingFrequency, self.recordDuration, frames, self.flags.copy())

    def addWhiteNoise(self, amplitude: float = 0.1) -> "TimeSeries":
        """
        Return a copy with Gaussian noise added to all joint values.

        @param amplitude: Standard deviation of the noise in degrees.
        @type amplitude: float
        @rtype: TimeSeries
        """
        new = []
        for frame in self.jointPosition:
            size    = len(frame)
            samples = np.random.normal(0, amplitude, size=size)
            newFrame = {key: frame[key] + samples[i] for i, key in enumerate(frame)}
            new.append(newFrame)
        return TimeSeries(self.samplingFrequency, self.recordDuration, new, self.flags)

    def speed(self, factor: float) -> "TimeSeries":
        """
        Return a time-scaled copy of this series.

        A factor > 1 speeds up; a factor < 1 slows down.

        @param factor: Speed multiplier (must be strictly positive).
        @type factor: float
        @rtype: TimeSeries
        @raise ValueError: If factor is not positive.
        """
        if factor <= 0:
            raise ValueError("Speed factor must be strictly positive.")
        original = self.jointPosition
        if not original:
            return TimeSeries(self.samplingFrequency, self.recordDuration, [], self.flags.copy())

        keys      = list(original[0].keys())
        nOriginal = len(original)
        nNew      = max(1, round(nOriginal / factor))
        newFrames = []

        for i in range(nNew):
            t     = i * (nOriginal - 1) / max(nNew - 1, 1)
            lo    = int(t)
            hi    = min(lo + 1, nOriginal - 1)
            alpha = t - lo
            frame = {k: original[lo][k] * (1 - alpha) + original[hi][k] * alpha for k in keys}
            newFrames.append(frame)

        return TimeSeries(self.samplingFrequency, self.recordDuration / factor, newFrames, self.flags.copy())

    def smooth(self, window: int = 5) -> "TimeSeries":
        """
        Return a copy with joint values smoothed by a sliding mean window.

        @param window: Window size (must be at least 2).
        @type window: int
        @rtype: TimeSeries
        @raise ValueError: If window is less than 2.
        """
        if window < 2:
            raise ValueError("Smoothing window must be at least 2.")
        original = self.jointPosition
        if not original:
            return TimeSeries(self.samplingFrequency, self.recordDuration, [], self.flags.copy())

        keys      = list(original[0].keys())
        n         = len(original)
        half      = window // 2
        newFrames = []

        for i in range(n):
            lo    = max(0, i - half)
            hi    = min(n, i + half + 1)
            frame = {k: sum(original[j][k] for j in range(lo, hi)) / (hi - lo) for k in keys}
            newFrames.append(frame)

        return TimeSeries(self.samplingFrequency, self.recordDuration, newFrames, self.flags.copy())

    def toDict(self) -> dict:
        """
        Serialize the time series to a plain dictionary.

        @rtype: dict
        """
        return {
            "samplingFrequency": self.samplingFrequency,
            "recordDuration":    self.recordDuration,
            "jointPosition":     self.jointPosition,
            "flags":             self.flags,
        }

    def saveToJson(self, fileName: str) -> None:
        """
        Save the time series to a JSON file.

        @param fileName: Output file path.
        @type fileName: str
        """
        cm.MKprint(f"Saving time series as JSON at: {fileName}", CLASS_NAME, CLASS_COLOR)
        with open(fileName, mode="w") as f:
            json.dump(self.toDict(), f, indent=4)

    def saveToCSV(self, fileName: str) -> None:
        """
        Save the time series to a CSV file.

        @param fileName: Output file path.
        @type fileName: str
        """
        cm.MKprint(f"Saving time series as CSV at: {fileName}", CLASS_NAME, CLASS_COLOR)
        with open(fileName, 'w', newline='') as csvFile:
            writer = csv.writer(csvFile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
            writer.writerow(self.jointLabel)
            for i, frame in enumerate(self.jointPosition):
                timestamp = i / self.samplingFrequency
                row = [i, timestamp] + [frame.get(joint, 0) for joint in self.jointLabel[2:]]
                writer.writerow(row)

    @classmethod
    def loadFromJson(cls, fileName: str) -> "TimeSeries":
        """
        Load a time series from a JSON file.

        @param fileName: Input file path.
        @type fileName: str
        @rtype: TimeSeries
        """
        cm.MKprint(f"Loading time series from JSON at: {fileName}", CLASS_NAME, CLASS_COLOR)
        with open(fileName, mode="r") as f:
            data = json.load(f)
        return cls(
            data["samplingFrequency"],
            data["recordDuration"],
            data["jointPosition"],
            data.get("flags", [True, True, True])
        )

    @classmethod
    def loadFromCSV(cls, fileName: str) -> "TimeSeries":
        """
        Load a time series from a CSV file.

        @param fileName: Input file path.
        @type fileName: str
        @rtype: TimeSeries
        """
        cm.MKprint(f"Loading time series from CSV at: {fileName}", CLASS_NAME, CLASS_COLOR)
        jointPosition = []
        exclude       = {"frame", "timestamp"}
        motorLabels   = [l for l in cls.jointLabel if l not in exclude]

        with open(fileName, newline='') as csvFile:
            reader        = csv.reader(csvFile, delimiter=',', quotechar='|')
            header        = next(reader)
            df            = pd.read_csv(fileName)
            rdIdx         = df["timestamp"].iloc[-1]
            sfIdx         = df["frame"].iloc[-1] / rdIdx
            timestampIdx  = header.index("timestamp")
            jointIndices  = {name: header.index(name) for name in motorLabels}

            for row in reader:
                frame               = {}
                frame["timestamp"]  = float(row[timestampIdx])
                for joint, idx in jointIndices.items():
                    frame[joint] = float(row[idx])
                jointPosition.append(frame)

        return cls(float(sfIdx), float(rdIdx), jointPosition, [True, True, True])

    def plot(self) -> None:
        """
        Display the time series as a 3x3 grid of motor-angle plots.

        Subplots for inactive channels are hidden based on L{flags}.

        @raise ValueError: If the series has no frames.
        """
        if not self.jointPosition:
            raise ValueError("jointPosition is empty.")

        sample = {name: [] for name in self.jointPosition[0].keys()}
        for frame in self.jointPosition:
            for name in frame.keys():
                sample[name].append(frame[name])

        sampleCount = min(len(self.jointPosition), *(len(v) for v in sample.values()))
        time_axis   = [i / self.samplingFrequency for i in range(sampleCount)]
        for key in sample:
            sample[key] = sample[key][:sampleCount]

        fig, axs = plt.subplots(3, 3, figsize=(15, 10))
        axs = axs.flatten()

        subplotMapping = [
            (2, config.TIME_SERIE_HEAD_VALUES_NAME,
             "Head looking point"),
            (0, [str(config.ARM_RIGHT_ID) + "_" + m for m in config.SHOULDER_MOTOR_NAME],
             "Right shoulder motors angles"),
            (3, [str(config.ARM_RIGHT_ID) + "_" + m for m in config.ELBOW_MOTOR_NAME],
             "Right elbow motors angles"),
            (6, [str(config.ARM_RIGHT_ID) + "_" + m for m in config.FOREARM_MOTOR_NAME],
             "Right forearm motors angles"),
            (1, [str(config.ARM_LEFT_ID) + "_" + m for m in config.SHOULDER_MOTOR_NAME],
             "Left shoulder motors angles"),
            (4, [str(config.ARM_LEFT_ID) + "_" + m for m in config.ELBOW_MOTOR_NAME],
             "Left elbow motors angles"),
            (7, [str(config.ARM_LEFT_ID) + "_" + m for m in config.FOREARM_MOTOR_NAME],
             "Left forearm motors angles"),
        ]

        for idx, motors, title in subplotMapping:
            ax = axs[idx]
            for m in motors:
                if m in sample:
                    ax.plot(time_axis, sample[m], label=m)
            ax.set_title(title)
            ax.legend(fontsize=8)
            ax.grid(True)

        axs[8].axis("off")
        axs[5].axis("off")

        if not self.flags[0]:
            axs[0].axis("off");  axs[3].axis("off");  axs[6].axis("off")
        if not self.flags[1]:
            axs[1].axis("off");  axs[4].axis("off");  axs[7].axis("off")
        if not self.flags[2]:
            axs[2].axis("off")

        plt.tight_layout()
        plt.show()

    @staticmethod
    def _toNumpy(ts: "TimeSeries") -> tuple:
        """
        Convert a time series to a (channels x frames) numpy array.

        @param ts: Input time series.
        @type ts: TimeSeries
        @rtype: tuple[numpy.ndarray, list]
        @return: (array, key_list)
        """
        keys = list(ts.jointPosition[0].keys())
        arr  = np.array([[frame.get(k, 0.0) for frame in ts.jointPosition] for k in keys])
        return arr, keys

    @staticmethod
    def _fromNumpy(arr: np.ndarray, keys: list, samplingFrequency: float,
                   flags: list) -> "TimeSeries":
        """
        Convert a (channels x frames) numpy array back to a TimeSeries.

        @param arr: Numpy array of shape (n_channels, n_frames).
        @type arr: numpy.ndarray
        @param keys: Channel names in row order.
        @type keys: list
        @param samplingFrequency: Sampling frequency in Hz.
        @type samplingFrequency: float
        @param flags: Active channel flags.
        @type flags: list
        @rtype: TimeSeries
        """
        nFrames  = arr.shape[1]
        frames   = [{keys[c]: float(arr[c, i]) for c in range(len(keys))} for i in range(nFrames)]
        duration = nFrames / samplingFrequency
        return TimeSeries(samplingFrequency, duration, frames, flags)

    @staticmethod
    def dba(seriesList: list, nIterations: int = 10) -> "TimeSeries":
        """
        Compute the DTW Barycenter Average of a list of time series.

        @param seriesList: List of TimeSeries instances.
        @type seriesList: list[TimeSeries]
        @param nIterations: Number of DBA iterations.
        @type nIterations: int
        @rtype: TimeSeries
        @raise ValueError: If the list is empty or frequencies differ.
        """
        if not seriesList:
            raise ValueError("seriesList must not be empty.")
        if len(seriesList) == 1:
            return seriesList[0]

        freqs = [ts.samplingFrequency for ts in seriesList]
        for i in range(len(freqs) - 1):
            if abs(freqs[i] - freqs[i + 1]) >= 0.1:
                raise ValueError("All TimeSeries must share the same samplingFrequency.")

        sf        = seriesList[0].samplingFrequency
        converted = [TimeSeries._toNumpy(ts) for ts in seriesList]
        numpySeries = [arr for arr, keys in converted]
        keys        = converted[0][1]
        averageArr  = performDBA(numpySeries, n_iterations=nIterations)

        flags = [any(ts.flags[i] for ts in seriesList) for i in range(3)]
        return TimeSeries._fromNumpy(averageArr, keys, sf, flags)
