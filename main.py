from reachy_sdk import ReachySDK
from reachyController import ReachyController
import time


def main():
    reachy  = ReachySDK(host='localhost')
    reachyC = ReachyController(reachy)

    reachyC.turnOn()
    reachyC.armLeft._debug_placeHandOnTable()

    future = reachyC.runAsync(reachyC.armLeft.gotoCartesianPoint, [2, 0.19, 0], [0, -90, 0], 5)
    a = reachyC.record(5, 20)
    future.result()

    time.sleep(1)

    
    reachyC.runAsync(reachyC.playRecord,a)
    time.sleep(3)
    b = reachyC.record(5, 20)

    future.result()
    reachyC.turnOffSmooth()
    a.plot()
    b.plot()

if __name__ == "__main__":
    main()
