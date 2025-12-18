import json
import matplotlib.pyplot as plt

import numpy as np

import headController as hc
import armController as arm

class TimeSeries():

    def __init__(self, samplingFrequency : float, recordDurationSeconds : float, jointPosition : list = []):
        self.samplingFrequency = samplingFrequency
        self.recordDuration = recordDurationSeconds
        self.jointPosition = jointPosition
    
    def __add__(self, ts : "TimeSeries"):
        if self.samplingFrequency != ts.samplingFrequency:
            raise("you cannot add time series with different sampling frequency /!\\")
        def addList(d0 : list, d1 : list) -> list:
            if len(d0) < len(d1):
                d1, d0 = d0, d1
            for i in range(len(d1)):
                d0[i].update(d1[i])
            return d0

        
        duration : float = max(self.recordDuration, ts.recordDuration)
        r : "TimeSeries" = TimeSeries(self.samplingFrequency, duration, addList(self.jointPosition, ts.jointPosition))

        return r
    
    def getDictFromTimeSerie(self) -> dict:
        return {"samplingFrequency" : self.samplingFrequency, "recordDuration" : self.recordDuration, "jointPosition" : self.jointPosition}

    def dictToJson(self, dict : dict) -> str:
        return json.dumps(dict, indent=4)
    
    def jsonToDict(self, jsonString : str) -> dict:
        return json.loads(jsonString)
    
    def saveRecordsInJson(self, fileName : str) -> None:
        file = open(fileName, mode="w")
        if not file:
            raise "Cannot open file " + fileName
        
        file.write(self.dictToJson(self.getDictFromTimeSerie()))
        file.close()

    def loadRecordsFromJson(self, fileName : str) -> dict:
        file = open(fileName, mode="r")
        if not file:
            raise "Cannot open file " + fileName
        dictJson = self.jsonToDict(file.read())
        return dictJson
    
    def plot(self):
        if not self.jointPosition:
            raise ValueError("jointPosition is empty")

        sample = {name: [] for name in self.jointPosition[0].keys()}
        for frame in self.jointPosition:
            for name in frame.keys():
                sample[name].append(frame[name])

        sampleCount = min(len(self.jointPosition), *(len(v) for v in sample.values()))
        time = [i / self.samplingFrequency for i in range(sampleCount)]

        for key in sample:
            sample[key] = sample[key][:sampleCount]

        fig, axs = plt.subplots(3, 3, figsize=(15, 10))
        axs = axs.flatten()

        subplotMapping = [
            (2, hc.ReachyHead.DISK_MOTOR_NAME, "Head motors angles"),
            (5, hc.ReachyHead.ANTENNA_MOTOR_NAME, "Antenna motors angles"),
            (0, ["r_" + i for i in arm.ReachyArm.SHOULDER_MOTOR_NAME], "Right shoulder motors angles"),
            (3, ["r_" + i for i in arm.ReachyArm.ELBOW_MOTOR_NAME], "Right elbow motors angles"),
            (6, ["r_" + i for i in arm.ReachyArm.FOREARM_MOTOR_NAME], "Right forearm motors angles"),
            (1, ["l_" + i for i in arm.ReachyArm.SHOULDER_MOTOR_NAME], "Left shoulder motors angles"),
            (4, ["l_" + i for i in arm.ReachyArm.ELBOW_MOTOR_NAME], "Left elbow motors angles"),
            (7, ["l_" + i for i in arm.ReachyArm.FOREARM_MOTOR_NAME], "Left forearm motors angles"),
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
