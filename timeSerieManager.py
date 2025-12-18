class TimeSeries():

    def __init__(self, samplingFrequency : float, recordDurationSeconds : float, jointPosition : list = []):
        self.samplingFrequency = samplingFrequency
        self.recordDuration = recordDurationSeconds
        self.jointPosition = jointPosition
    
    def __add__(self, ts : "TimeSeries"):
        if self.samplingFrequency != ts.samplingFrequency:
            raise("you cannot add time series with different sampling frequency /!\\")
        def addList(d0 : list, d1 : list):
            if len(d0) < len(d1):
                d1, d0 = d0, d1
            for i in range(len(d1)):
                d0[i].update(d1[i])

        
        duration : float = max(self.recordDuration, ts.recordDuration)
        r : "TimeSeries" = TimeSeries(self.samplingFrequency, duration, addList(self.jointPosition, ts.jointPosition))