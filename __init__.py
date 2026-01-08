from reachy_sdk import ReachySDK
import armController as ac
import headController as hc
import consoleManager as cm
import timeSerieManager as ts
from concurrent.futures import ThreadPoolExecutor


class ReachyController():
    #arm id
    ARM_LEFT_ID : str = 'l'
    ARM_RIGHT_ID : str = 'r'

    #console
    CLASS_NAME : str = "Reachy controller"
    CLASS_COLOR : str = cm.Color.BRIGHT_RED

    _executor = ThreadPoolExecutor(max_workers=10)

    def __init__(self, reachy : ReachySDK):
        self.reachy = reachy
        self.armLeft = ac.ReachyArm(self.reachy, ReachyController.ARM_LEFT_ID)
        self.armRight = ac.ReachyArm(self.reachy, ReachyController.ARM_RIGHT_ID)
        self.head = hc.ReachyHead(self.reachy)

    def runAsync(self, func, *args):
        self._executor.submit(func, *args)

    def record(self, recordDurationSeconds : float, samplingFrequencyHertz : float, recordArmLeft : bool = True, recordArmRight : bool = True, recordHead : bool = True) -> "ts.TimeSeries":
        """
        record all part of reachy for a set duration with a sampling frequency of samplingFrequencyHertz Hz. rightArm and leftArm control if an arm is recorded or not; true -> recorded, false -> ignored
        PARAMETER samplingFrenquencyHertz TYPE float, duration TYPE float, recordArmRight TYPE bool, recordArmLeft TYPE bool, recordHead TYPE bool
        RETURN dict
        """

        cm.MKprint("|-------------------- Start recording reachy joints ------------------|", ReachyController.CLASS_NAME, ReachyController.CLASS_COLOR)
        cm.MKprint("recording left hand : " + str(recordArmLeft) + ", recording right hand : " + str(recordArmRight) + ", recording head : " + str(recordHead), ReachyController.CLASS_NAME, ReachyController.CLASS_COLOR)

        with ThreadPoolExecutor(max_workers=3) as executor:
            if recordArmLeft:
                future_l = executor.submit(self.armLeft.recordArm, recordDurationSeconds, samplingFrequencyHertz)
            if recordArmRight:
                future_r = executor.submit(self.armRight.recordArm, recordDurationSeconds, samplingFrequencyHertz)
            if recordHead:
                future_h = executor.submit(self.head.recordHead, recordDurationSeconds, samplingFrequencyHertz)

            if recordArmLeft:
                lArmRecord = future_l.result()
            if recordArmRight:
                rArmRecord = future_r.result()
            if recordHead:
                headRecord = future_h.result()

        r : "ts.TimeSeries"
        #say hello to my ugly as hell code
        if recordArmLeft or recordArmRight or recordHead:
            if recordArmLeft:
                r = lArmRecord
                if recordArmRight:
                    r = r + rArmRecord
                if recordHead:
                    r = r + headRecord
            
            elif recordArmRight:
                r = rArmRecord
                if recordArmLeft:
                    r = r + lArmRecord
                if recordHead:
                    r = r + headRecord
            
            elif recordHead:
                r = headRecord
                if recordArmLeft:
                    r = r + lArmRecord
                if recordArmRight:
                    r = r + rArmRecord
        else:
            raise("you need to record at least one part !")
        
        cm.MKprint("|-------------------- Stop recording reachy joints ------------------|", ReachyController.CLASS_NAME, ReachyController.CLASS_COLOR)

        return r
    
    def playRecord(self, records : "ts.TimeSeries", startDuration : float = 3.0) -> None:
        """
        play record generated from self.record
        PARAMETER records TYPE ts.TimeSeries, startDuration TYPE float
        RETURN None
        """
        cm.MKprint("|-------------------- Start playing reachy records ------------------|", ReachyController.CLASS_NAME, ReachyController.CLASS_COLOR)
        with ThreadPoolExecutor(max_workers=3) as executor:
            executor.submit(self.armLeft.playArmRecord, records, startDuration)
            executor.submit(self.armRight.playArmRecord, records, startDuration)
            executor.submit(self.head.playHeadRecord, records, startDuration)
        cm.MKprint("|-------------------- Stop playing reachy records ------------------|", ReachyController.CLASS_NAME, ReachyController.CLASS_COLOR)

if __name__ == "__main__":
    reachy = ReachySDK(host='localhost')
    reachyC = ReachyController(reachy)
    reachyC.runAsync(reachyC.armRight.openHand, 5)
    reachyC.runAsync(reachyC.armLeft.openHand, 5)
    #reachyC.runAsync(reachyC.armRight.gotoCartesianPoint, [3, 5, -2], [0, -90, 0], 5)
    a = reachyC.record(5, 20, True, True, False)
    a.plot()
    reachyC.playRecord(a)