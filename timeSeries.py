import json
import matplotlib.pyplot as plt
import config


class TimeSeries:

    def __init__(self, samplingFrequency: float, recordDurationSeconds: float, jointPosition: list = None):
        self.samplingFrequency  = samplingFrequency
        self.recordDuration     = recordDurationSeconds
        self.jointPosition      = jointPosition if jointPosition is not None else []

    def __add__(self, other: "TimeSeries") -> "TimeSeries":
        if self.samplingFrequency != other.samplingFrequency:
            raise ValueError("Cannot merge TimeSeries with different sampling frequencies.")

        def mergeLists(a: list, b: list) -> list:
            a = [frame.copy() for frame in a]
            b = [frame.copy() for frame in b]
            if len(a) < len(b):
                a, b = b, a
            for i in range(len(b)):
                a[i].update(b[i])
            return a

        duration = max(self.recordDuration, other.recordDuration)
        return TimeSeries(self.samplingFrequency, duration, mergeLists(self.jointPosition, other.jointPosition))

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

    @classmethod
    def loadFromJson(cls, fileName: str) -> "TimeSeries":
        with open(fileName, mode="r") as f:
            data = json.load(f)
        return cls(data["samplingFrequency"], data["recordDuration"], data["jointPosition"])

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
