from reachy_sdk import ReachySDK
import armController as ac
import headController as hc
import consoleManager as cm
from concurrent.futures import ThreadPoolExecutor
import time

class ReachyController():
    #arm id
    ARM_LEFT_ID : str = 'l'
    ARM_RIGHT_ID : str = 'r'

    #console
    CLASS_NAME : str = "Reachy controller"
    CLASS_COLOR : str = cm.Color.BRIGHT_RED

    def __init__(self, reachy : ReachySDK):
        self.reachy = reachy
        self.armLeft = ac.ReachyArm(self.reachy, ReachyController.ARM_LEFT_ID)
        self.armRight = ac.ReachyArm(self.reachy, ReachyController.ARM_RIGHT_ID)
        self.head = hc.ReachyHead(self.reachy)

    def record(self, recordDurationSeconds : float, samplingFrequencyHertz : float, recordArmLeft : bool = True, recordArmRight : bool = True, recordHead : bool = True) -> dict:
        """
        record all part of reachy for a set duration with a sampling frequency of samplingFrequencyHertz Hz. rightArm and leftArm control if an arm is recorded or not; true -> recorded, false -> ignored
        PARAMETER samplingFrenquencyHertz TYPE float, duration TYPE float, recordArmRight TYPE bool, recordArmLeft TYPE bool, recordHead TYPE bool
        RETURN dict
        """
        start : float = time.time()
        r : dict = {"samplingFrequency" : samplingFrequencyHertz, "recordDuration" : recordDurationSeconds, "startTime" : start}
        
        cm.MKprint("|-------------------- Start recording reachy joints ------------------|", ReachyController.CLASS_NAME, ReachyController.CLASS_COLOR)
        cm.MKprint("recording left hand : " + recordArmLeft + ", recording right hand : " + recordArmRight + ", recording head : " + recordHead, ReachyController.CLASS_NAME, ReachyController.CLASS_COLOR)

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

        
        if recordArmLeft:
            r["armLeftRecords"] = {"jointPosition" : lArmRecord["jointPosition"], "handPosition" : lArmRecord["handPosition"]}
        if recordArmRight:
            r["armRightRecords"] = {"jointPosition" : rArmRecord["jointPosition"], "handPosition" : rArmRecord["handPosition"]}
        if recordHead:
            r["headRecords"] = {"diskPosition" : headRecord["diskPosition"]}
        
        cm.MKprint("|-------------------- Stop recording reachy joints ------------------|", ReachyController.CLASS_NAME, ReachyController.CLASS_COLOR)

        return r
    
