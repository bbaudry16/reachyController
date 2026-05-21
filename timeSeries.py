import json
import matplotlib.pyplot as plt
import pandas as pd
from . import config
import csv


class TimeSeries:

    JOINT_LABLE : list = ["frame","timestamp","r_shoulder_pitch","r_shoulder_roll","r_arm_yaw","r_elbow_pitch","r_forearm_yaw","r_wrist_pitch","r_wrist_roll","r_gripper","l_shoulder_pitch","l_shoulder_roll","l_arm_yaw","l_elbow_pitch","l_forearm_yaw","l_wrist_pitch","l_wrist_roll","l_gripper","head_x","head_y","head_z"]

    def __init__(self, samplingFrequency: float, recordDurationSeconds: float, jointPosition: list = None):
        self.samplingFrequency  = samplingFrequency
        self.recordDuration     = recordDurationSeconds
        self.jointPosition      = jointPosition if jointPosition is not None else []

    def mergeLists(self, a: list, b: list) -> list:
        a = [frame.copy() for frame in a]
        b = [frame.copy() for frame in b]
        if len(a) < len(b):
            a, b = b, a
        for i in range(len(b)):
            a[i].update(b[i])
        return a

    # ─── Operators ─────────────────────────────────────────────────────────────

    def __add__(self, other: "TimeSeries") -> "TimeSeries":
        """Merge two TimeSeries recorded at the same time (parallel joints)."""
        if self.samplingFrequency != other.samplingFrequency:
            raise ValueError("Cannot merge TimeSeries with different sampling frequencies.")
        duration = max(self.recordDuration, other.recordDuration)
        return TimeSeries(self.samplingFrequency, duration, self.mergeLists(self.jointPosition, other.jointPosition))

    def __rshift__(self, other: "TimeSeries") -> "TimeSeries":
        """
        Concatenate two TimeSeries end-to-end: self plays first, then other.
        PARAMETER other : TimeSeries
        RETURN TimeSeries
        """
        if self.samplingFrequency != other.samplingFrequency:
            raise ValueError("Cannot concatenate TimeSeries with different sampling frequencies.")
        frames    = [f.copy() for f in self.jointPosition] + [f.copy() for f in other.jointPosition]
        duration  = self.recordDuration + other.recordDuration
        return TimeSeries(self.samplingFrequency, duration, frames)

    def __getitem__(self, key) -> "TimeSeries":
        """
        Extract a sub-sequence by frame slice: ts[10:50], ts[:100], ts[50:].
        PARAMETER key : slice
        RETURN TimeSeries
        """
        if not isinstance(key, slice):
            raise TypeError("TimeSeries only supports slice indexing, e.g. ts[10:50].")
        frames   = [f.copy() for f in self.jointPosition[key]]
        duration = len(frames) / self.samplingFrequency
        return TimeSeries(self.samplingFrequency, duration, frames)

    def __mul__(self, n: int) -> "TimeSeries":
        """
        Repeat the TimeSeries n times end-to-end: ts * 3.
        PARAMETER n : int
        RETURN TimeSeries
        """
        if not isinstance(n, int) or n < 1:
            raise ValueError("Repeat count must be a positive integer.")
        frames   = [f.copy() for f in self.jointPosition] * n
        duration = self.recordDuration * n
        return TimeSeries(self.samplingFrequency, duration, frames)

    def __rmul__(self, n: int) -> "TimeSeries":
        return self.__mul__(n)

    def __len__(self) -> int:
        return len(self.jointPosition)

    # ─── Transformations ───────────────────────────────────────────────────────

    def reverse(self) -> "TimeSeries":
        """
        Return a new TimeSeries with frames in reverse order.
        PARAMETER None
        RETURN TimeSeries
        """
        frames = [f.copy() for f in reversed(self.jointPosition)]
        return TimeSeries(self.samplingFrequency, self.recordDuration, frames)

    def speed(self, factor: float) -> "TimeSeries":
        """
        Return a new TimeSeries resampled at a different speed.
        factor > 1 speeds up (fewer frames), factor < 1 slows down (more frames).
        Uses linear interpolation between frames.
        PARAMETER factor : float
        RETURN TimeSeries
        """
        if factor <= 0:
            raise ValueError("Speed factor must be strictly positive.")

        original = self.jointPosition
        if not original:
            return TimeSeries(self.samplingFrequency, self.recordDuration, [])

        keys          = list(original[0].keys())
        n_original    = len(original)
        n_new         = max(1, round(n_original / factor))
        new_frames    = []

        for i in range(n_new):
            t        = i * (n_original - 1) / max(n_new - 1, 1)
            lo       = int(t)
            hi       = min(lo + 1, n_original - 1)
            alpha    = t - lo
            frame    = {}
            for k in keys:
                frame[k] = original[lo][k] * (1 - alpha) + original[hi][k] * alpha
            new_frames.append(frame)

        new_duration = self.recordDuration / factor
        return TimeSeries(self.samplingFrequency, new_duration, new_frames)

    def smooth(self, window: int = 5) -> "TimeSeries":
        """
        Return a new TimeSeries with joint positions smoothed by a moving average.
        window : number of frames to average (odd recommended, minimum 2).
        Edges are handled by shrinking the window (no padding artefacts).
        PARAMETER window : int
        RETURN TimeSeries
        """
        if window < 2:
            raise ValueError("Smoothing window must be at least 2.")

        original = self.jointPosition
        if not original:
            return TimeSeries(self.samplingFrequency, self.recordDuration, [])

        keys       = list(original[0].keys())
        n          = len(original)
        half       = window // 2
        new_frames = []

        for i in range(n):
            lo    = max(0, i - half)
            hi    = min(n, i + half + 1)
            frame = {}
            for k in keys:
                frame[k] = sum(original[j][k] for j in range(lo, hi)) / (hi - lo)
            new_frames.append(frame)

        return TimeSeries(self.samplingFrequency, self.recordDuration, new_frames)

    # ─── Serialization ─────────────────────────────────────────────────────────

    def toDict(self) -> dict:
        return {
            "samplingFrequency": self.samplingFrequency,
            "recordDuration":    self.recordDuration,
            "jointPosition":     self.jointPosition,
        }

    def saveToJson(self, fileName: str) -> None:
        with open(fileName, mode="w") as f:
            json.dump(self.toDict(), f, indent=4)

    def saveToCSV(self, fileName: str) -> None:
        headers = self.JOINT_LABLE
        with open(fileName, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
            writer.writerow(headers)
            for i, frame in enumerate(self.jointPosition):
                timestamp = i / self.samplingFrequency
                row = [i, timestamp]
                for joint in self.JOINT_LABLE[2:]:
                    row.append(frame.get(joint, 0))
                writer.writerow(row)

    @classmethod
    def loadFromJson(cls, fileName: str) -> "TimeSeries":
        with open(fileName, mode="r") as f:
            data = json.load(f)
        return cls(data["samplingFrequency"], data["recordDuration"], data["jointPosition"])

    @classmethod
    def loadFromCSV(cls, fileName: str) -> "TimeSeries":
        jointPosition = []
        EXCLUDE       = {"frame", "timestamp", "head_x", "head_y", "head_z"}
        MOTOR_LABELS  = [l for l in cls.JOINT_LABLE if l not in EXCLUDE]

        with open(fileName, newline='') as csvfile:
            reader = csv.reader(csvfile, delimiter=',', quotechar='|')
            header = next(reader)

            df     = pd.read_csv(fileName)
            rd_idx = df["timestamp"].iloc[-1]
            sf_idx = df["frame"].iloc[-1] / rd_idx

            timestamp_idx = header.index("timestamp")
            joint_indices = {name: header.index(name) for name in MOTOR_LABELS}

            for row in reader:
                frame             = {}
                frame["timestamp"] = float(row[timestamp_idx])
                for joint, idx in joint_indices.items():
                    frame[joint] = float(row[idx])
                jointPosition.append(frame)

            return cls(float(sf_idx), float(rd_idx), jointPosition)

    # ─── Plot ──────────────────────────────────────────────────────────────────

    def plot(self):
        if not self.jointPosition:
            raise ValueError("jointPosition is empty.")

        sample = {name: [] for name in self.jointPosition[0].keys()}
        for frame in self.jointPosition:
            for name in frame.keys():
                sample[name].append(frame[name])

        sampleCount = min(len(self.jointPosition), *(len(v) for v in sample.values()))
        time        = [i / self.samplingFrequency for i in range(sampleCount)]

        for key in sample:
            sample[key] = sample[key][:sampleCount]

        fig, axs = plt.subplots(3, 3, figsize=(15, 10))
        axs = axs.flatten()

        subplotMapping = [
            (2, config.DISK_MOTOR_NAME,                                    "Head motors angles"),
            (5, config.ANTENNA_MOTOR_NAME,                                 "Antenna motors angles"),
            (0, ["r_" + m for m in config.SHOULDER_MOTOR_NAME],           "Right shoulder motors angles"),
            (3, ["r_" + m for m in config.ELBOW_MOTOR_NAME],              "Right elbow motors angles"),
            (6, ["r_" + m for m in config.FOREARM_MOTOR_NAME],            "Right forearm motors angles"),
            (1, ["l_" + m for m in config.SHOULDER_MOTOR_NAME],           "Left shoulder motors angles"),
            (4, ["l_" + m for m in config.ELBOW_MOTOR_NAME],              "Left elbow motors angles"),
            (7, ["l_" + m for m in config.FOREARM_MOTOR_NAME],            "Left forearm motors angles"),
        ]

        for idx, motors, title in subplotMapping:
            ax = axs[idx]
            for m in motors:
                if m in sample:
                    ax.plot(time, sample[m], label=m)
            ax.set_title(title)
            ax.legend(fontsize=8)
            ax.grid(True)

        axs[8].axis("off")
        plt.tight_layout()
        plt.show()

    # ─── DBA (DTW Barycenter Averaging) via fpetitjean/DBA ────────────────────────

    @staticmethod
    def _to_numpy(ts: "TimeSeries") -> "np.ndarray":
        """
        Convert a TimeSeries to a numpy array of shape (n_channels, length)
        as expected by DBA_multivariate.performDBA (channels-first format).
        Only joints present in the first frame are used; 'timestamp' is excluded.
        PARAMETER ts : TimeSeries
        RETURN np.ndarray shape (n_channels, length)
        """
        import numpy as np
        keys = [k for k in ts.jointPosition[0].keys() if k != "timestamp"]
        return np.array([[frame[k] for frame in ts.jointPosition] for k in keys])

    @staticmethod
    def _from_numpy(arr: "np.ndarray", keys: list, samplingFrequency: float) -> "TimeSeries":
        """
        Convert a numpy array of shape (n_channels, length) back to a TimeSeries.
        PARAMETER arr : np.ndarray, keys : list[str], samplingFrequency : float
        RETURN TimeSeries
        """
        n_frames = arr.shape[1]
        frames   = [{keys[c]: float(arr[c, i]) for c in range(len(keys))} for i in range(n_frames)]
        duration = n_frames / samplingFrequency
        return TimeSeries(samplingFrequency, duration, frames)

    @staticmethod
    def dba(series_list: list, n_iterations: int = 10) -> "TimeSeries":
        """
        Compute the DTW Barycenter Average of a list of TimeSeries
        using fpetitjean/DBA (DBA_multivariate.performDBA).

        Series do NOT need to have the same length — DBA handles variable-length
        inputs natively. All series must share the same samplingFrequency and
        the same joint keys.

        PARAMETER series_list : list[TimeSeries], n_iterations : int
        RETURN TimeSeries
        """
        from .DBA_multivariate import performDBA

        if not series_list:
            raise ValueError("series_list must not be empty.")
        if len(series_list) == 1:
            return series_list[0]

        freqs = {ts.samplingFrequency for ts in series_list}
        if len(freqs) > 1:
            raise ValueError("All TimeSeries must share the same samplingFrequency.")

        sf   = series_list[0].samplingFrequency
        keys = [k for k in series_list[0].jointPosition[0].keys() if k != "timestamp"]

        numpy_series = [TimeSeries._to_numpy(ts) for ts in series_list]
        average_arr  = performDBA(numpy_series, n_iterations=n_iterations)

        return TimeSeries._from_numpy(average_arr, keys, sf)

    def __or__(self, other: "TimeSeries") -> "TimeSeries":
        """
        Average two TimeSeries via DBA: ts1 | ts2.
        Can be chained: ts1 | ts2 | ts3  (left-associative, re-runs DBA each step).
        For averaging many series at once, prefer TimeSeries.dba([...]).
        PARAMETER other : TimeSeries
        RETURN TimeSeries
        """
        return TimeSeries.dba([self, other])