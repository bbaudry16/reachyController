from reachy_sdk import ReachySDK
import armController as ac

class reachyController():

    ARM_LEFT_ID : str = 'l'
    ARM_RIGHT_ID : str = 'r'

    def __init__(self, reachy : ReachySDK):
        self.reachy = reachy
        self.armLeft = ac.ReachyArm(self.reachy, reachyController.ARM_LEFT_ID)
        self.armRight = ac.ReachyArm(self.reachy, reachyController.ARM_RIGHT_ID)