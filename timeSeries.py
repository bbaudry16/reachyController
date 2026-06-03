import json
import matplotlib.pyplot as plt
import pandas as pd
from . import config
from . import consoleManager as cm
import csv
import numpy as np
from .DBA_multivariate import performDBA

CLASS_NAME  : str = "Time serie"
CLASS_COLOR : str = cm.Color.BRIGHT_GREEN

class TimeSeries:


    rightJoint:list = [str(config.ARM_RIGHT_ID) + "_" + str(x) for x in config.ARM_MOTOR_NAME]

    leftJoint:list = [str(config.ARM_LEFT_ID) + "_" + str(x) for x in config.ARM_MOTOR_NAME]

    headJoint:list = ["head_x", "head_y", "head_z"]

    jointLabel:list = ["frame", "timestamp"] + rightJoint + leftJoint + headJoint

    def __init__(self,samplingFrequency:float,recordDurationSeconds:float,jointPosition:list=None,flags=[True,True,True]):
        self.samplingFrequency=samplingFrequency
        self.recordDuration=recordDurationSeconds
        self.jointPosition=jointPosition if jointPosition is not None else []
        self.flags=flags.copy()

    def mergeLists(self, a:list, b:list, aFlags:list, bFlags:list) -> tuple:

        maxLen=max(len(a),len(b))

        if not a:
            a=[{}]

        if not b:
            b=[{}]

        aLast=a[-1]
        bLast=b[-1]

        merged=[]

        for i in range(maxLen):

            frameA=a[i] if i<len(a) else aLast
            frameB=b[i] if i<len(b) else bLast
            frame=frameA.copy()

            if bFlags[0]:
                for joint in self.rightJoint:
                    if joint in frameB:
                        frame[joint]=frameB[joint]

            if bFlags[1]:
                for joint in self.leftJoint:
                    if joint in frameB:
                        frame[joint]=frameB[joint]

            if bFlags[2]:
                for joint in self.headJoint:
                    if joint in frameB:
                        frame[joint]=frameB[joint]

            if "timestamp" in frameB:
                frame["timestamp"]=frameB["timestamp"]

            merged.append(frame)

        mergedFlags=[aFlags[0] or bFlags[0],aFlags[1] or bFlags[1],aFlags[2] or bFlags[2]]

        return merged,mergedFlags

    def __add__(self,other:"TimeSeries")->"TimeSeries":

        if self.samplingFrequency!=other.samplingFrequency:
            raise ValueError("Cannot merge TimeSeries with different sampling frequencies.")

        duration=max(self.recordDuration,other.recordDuration)

        frames,flags=self.mergeLists(self.jointPosition,other.jointPosition,self.flags,other.flags)

        return TimeSeries(self.samplingFrequency,duration,frames,flags)

    def __rshift__(self,other:"TimeSeries")->"TimeSeries":

        if self.samplingFrequency!=other.samplingFrequency:
            raise ValueError("Cannot concatenate TimeSeries with different sampling frequencies.")

        frames=[f.copy() for f in self.jointPosition]+[f.copy() for f in other.jointPosition]
        duration=self.recordDuration+other.recordDuration
        flags=[self.flags[0] or other.flags[0],self.flags[1] or other.flags[1],self.flags[2] or other.flags[2]]

        return TimeSeries(self.samplingFrequency,duration,frames,flags)

    def __lshift__(self,other:"TimeSeries")->"TimeSeries":

        if self.samplingFrequency!=other.samplingFrequency:
            raise ValueError("Cannot concatenate TimeSeries with different sampling frequencies.")

        frames=[f.copy() for f in other.jointPosition]+[f.copy() for f in self.jointPosition]
        duration=other.recordDuration+self.recordDuration
        flags=[self.flags[0] or other.flags[0],self.flags[1] or other.flags[1],self.flags[2] or other.flags[2]]

        return TimeSeries(self.samplingFrequency,duration,frames,flags)

    def __getitem__(self,key)->"TimeSeries":

        if not isinstance(key,slice):
            raise TypeError("TimeSeries only supports slice indexing.")

        frames=[f.copy() for f in self.jointPosition[key]]
        duration=len(frames)/self.samplingFrequency

        return TimeSeries(self.samplingFrequency,duration,frames,self.flags.copy())

    def __mul__(self,n:int)->"TimeSeries":

        if not isinstance(n,int) or n<1:
            raise ValueError("Repeat count must be a positive integer.")

        frames=[f.copy() for f in self.jointPosition]*n
        duration=self.recordDuration*n

        return TimeSeries(self.samplingFrequency,duration,frames,self.flags.copy())

    def __rmul__(self,n:int)->"TimeSeries":
        return self.__mul__(n)

    def __len__(self)->int:
        return len(self.jointPosition)

    def __or__(self,other:"TimeSeries")->"TimeSeries":
        return TimeSeries.dba([self,other])

    def reverse(self)->"TimeSeries":

        frames=[f.copy() for f in reversed(self.jointPosition)]

        return TimeSeries(self.samplingFrequency,self.recordDuration,frames,self.flags.copy())

    def addWhiteNoise(self, amplitude : 0.1):

        mean = 0
        new = []
        for frame in self.jointPosition:
            newFrame = {}
            
            size = len(frame)
            samples = np.random.normal(mean, amplitude, size=size)

            for index, key in enumerate(frame):
                
                newFrame[key] = frame[key] + samples[index]

            new.append(newFrame)

        return TimeSeries(self.samplingFrequency, self.recordDuration, new, self.flags)


    def speed(self,factor:float)->"TimeSeries":

        if factor<=0:
            raise ValueError("Speed factor must be strictly positive.")

        original=self.jointPosition

        if not original:
            return TimeSeries(self.samplingFrequency,self.recordDuration,[],self.flags.copy())

        keys=list(original[0].keys())
        nOriginal=len(original)
        nNew=max(1,round(nOriginal/factor))
        newFrames=[]

        for i in range(nNew):

            t=i*(nOriginal-1)/max(nNew-1,1)
            lo=int(t)
            hi=min(lo+1,nOriginal-1)
            alpha=t-lo
            frame={}

            for k in keys:
                frame[k]=original[lo][k]*(1-alpha)+original[hi][k]*alpha

            newFrames.append(frame)

        newDuration=self.recordDuration/factor

        return TimeSeries(self.samplingFrequency,newDuration,newFrames,self.flags.copy())

    def smooth(self,window:int=5)->"TimeSeries":

        if window<2:
            raise ValueError("Smoothing window must be at least 2.")

        original=self.jointPosition

        if not original:
            return TimeSeries(self.samplingFrequency,self.recordDuration,[],self.flags.copy())

        keys=list(original[0].keys())
        n=len(original)
        half=window//2
        newFrames=[]

        for i in range(n):

            lo=max(0,i-half)
            hi=min(n,i+half+1)
            frame={}

            for k in keys:
                frame[k]=sum(original[j][k] for j in range(lo,hi))/(hi-lo)

            newFrames.append(frame)

        return TimeSeries(self.samplingFrequency,self.recordDuration,newFrames,self.flags.copy())

    def toDict(self)->dict:
        return {"samplingFrequency":self.samplingFrequency,"recordDuration":self.recordDuration,"jointPosition":self.jointPosition,"flags":self.flags}

    def saveToJson(self,fileName:str)->None:
        cm.MKprint("saving time serie as json at : " + fileName, CLASS_NAME, CLASS_COLOR)
        with open(fileName,mode="w") as f:
            json.dump(self.toDict(),f,indent=4)

    def saveToCSV(self,fileName:str)->None:

        headers=self.jointLabel

        with open(fileName,'w',newline='') as csvFile:

            writer=csv.writer(csvFile,delimiter=',',quotechar='|',quoting=csv.QUOTE_MINIMAL)
            writer.writerow(headers)
            
            cm.MKprint("saving time serie as CSV at : " + fileName, CLASS_NAME, CLASS_COLOR)

            for i,frame in enumerate(self.jointPosition):

                timestamp=i/self.samplingFrequency
                row=[i,timestamp]

                for joint in self.jointLabel[2:]:
                    row.append(frame.get(joint,0))

                writer.writerow(row)

    @classmethod
    def loadFromJson(cls,fileName:str)->"TimeSeries":

        with open(fileName,mode="r") as f:
            data=json.load(f)

        cm.MKprint("saving time serie as json at : " + fileName, CLASS_NAME, CLASS_COLOR)
        return cls(data["samplingFrequency"],data["recordDuration"],data["jointPosition"],data.get("flags",[True,True,True]))

    @classmethod
    def loadFromCSV(cls,fileName:str)->"TimeSeries":

        jointPosition=[]
        exclude={"frame","timestamp"}
        motorLabels=[l for l in cls.jointLabel if l not in exclude]

        with open(fileName,newline='') as csvFile:

            reader=csv.reader(csvFile,delimiter=',',quotechar='|')
            header=next(reader)
            df=pd.read_csv(fileName)
            rdIdx=df["timestamp"].iloc[-1]
            sfIdx=df["frame"].iloc[-1]/rdIdx
            timestampIdx=header.index("timestamp")
            jointIndices={name:header.index(name) for name in motorLabels}

            for row in reader:

                frame={}
                frame["timestamp"]=float(row[timestampIdx])

                for joint,idx in jointIndices.items():
                    frame[joint]=float(row[idx])

                jointPosition.append(frame)
        cm.MKprint("loading time serie from csv at : " + fileName, CLASS_NAME, CLASS_COLOR)
        return cls(float(sfIdx),float(rdIdx),jointPosition,[True,True,True])

    def plot(self):

        if not self.jointPosition:
            raise ValueError("jointPosition is empty.")

        sample={name:[] for name in self.jointPosition[0].keys()}

        for frame in self.jointPosition:
            for name in frame.keys():
                sample[name].append(frame[name])

        sampleCount=min(len(self.jointPosition),*(len(v) for v in sample.values()))
        time=[i/self.samplingFrequency for i in range(sampleCount)]

        for key in sample:
            sample[key]=sample[key][:sampleCount]

        fig,axs=plt.subplots(3,3,figsize=(15,10))
        axs=axs.flatten()



        subplotMapping = [
            (2, config.TIME_SERIE_HEAD_VALUES_NAME, "Head looking point"),
            (0, [str(config.ARM_RIGHT_ID) + "_" + m for m in config.SHOULDER_MOTOR_NAME], "Right shoulder motors angles"),
            (3, [str(config.ARM_RIGHT_ID) + "_" + m for m in config.ELBOW_MOTOR_NAME], "Right elbow motors angles"),
            (6, [str(config.ARM_RIGHT_ID) + "_" + m for m in config.FOREARM_MOTOR_NAME], "Right forearm motors angles"),
            (1, [str(config.ARM_LEFT_ID) + "_" + m for m in config.SHOULDER_MOTOR_NAME], "Left shoulder motors angles"),
            (4, [str(config.ARM_LEFT_ID) + "_" + m for m in config.ELBOW_MOTOR_NAME], "Left elbow motors angles"),
            (7, [str(config.ARM_LEFT_ID) + "_" + m for m in config.FOREARM_MOTOR_NAME], "Left forearm motors angles")
        ]
        for idx,motors,title in subplotMapping:

            ax=axs[idx]

            for m in motors:
                if m in sample:
                    ax.plot(time,sample[m],label=m)

            ax.set_title(title)
            ax.legend(fontsize=8)
            ax.grid(True)

        axs[8].axis("off")
        axs[5].axis("off")

        if not self.flags[0]:
            axs[0].axis("off")
            axs[3].axis("off")
            axs[6].axis("off")
        

        if not self.flags[1]:
            axs[1].axis("off")
            axs[4].axis("off")
            axs[7].axis("off")


        if not self.flags[2]:
            axs[2].axis("off")

        plt.tight_layout()
        plt.show()

    @staticmethod
    def _toNumpy(ts:"TimeSeries"):

        keys=list(ts.jointPosition[0].keys())

        return np.array([[frame.get(k,0.0) for frame in ts.jointPosition] for k in keys]),keys

    @staticmethod
    def _fromNumpy(arr:"np.ndarray",keys:list,samplingFrequency:float,flags:list)->"TimeSeries":

        nFrames=arr.shape[1]
        frames=[{keys[c]:float(arr[c,i]) for c in range(len(keys))} for i in range(nFrames)]
        duration=nFrames/samplingFrequency

        return TimeSeries(samplingFrequency,duration,frames,flags)

    @staticmethod
    def dba(seriesList:list,nIterations:int=10)->"TimeSeries":

        if not seriesList:
            raise ValueError("seriesList must not be empty.")

        if len(seriesList)==1:
            return seriesList[0]

        freqs=[ts.samplingFrequency for ts in seriesList]

        if len(freqs)>1:
            ok = True
            i = 0
            while(i < len(freqs) - 1 and ok):
                ok = abs(freqs[i] - freqs[i+1]) < 0.1
                i += 1

            if(not ok):
                raise ValueError("All TimeSeries must share the same samplingFrequency.")

        sf=seriesList[0].samplingFrequency
        converted=[TimeSeries._toNumpy(ts) for ts in seriesList]
        numpySeries=[arr for arr,keys in converted]
        keys=converted[0][1]
        averageArr=performDBA(numpySeries,n_iterations=nIterations)

        flags=[
            any(ts.flags[0] for ts in seriesList),
            any(ts.flags[1] for ts in seriesList),
            any(ts.flags[2] for ts in seriesList)
        ]

        return TimeSeries._fromNumpy(averageArr,keys,sf,flags)
