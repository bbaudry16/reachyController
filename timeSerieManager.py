import json

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