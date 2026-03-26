from reachy_sdk import ReachySDK
from reachyController import ReachyController
import time
import timeSeries as ts

def main():
    reachy  = ReachySDK(host='localhost')
    reachyC = ReachyController(reachy)

    reachyC.turnOn()
    reachyC.armLeft._debug_placeHandOnTable()

    future = reachyC.runAsync(reachyC.armLeft.gotoCartesianPoint, [2, 0.19, 0], [0, -90, 0], 5)
    a = reachyC.record(5, 20)
    future.result()

    reachyC.armLeft._debug_placeHandOnTable()
    a.plot()
    a.saveToJson("test.json")
    a.saveToCSV("test.csv")

    a.loadFromCSV("test.csv")
    reachyC.playRecord(a)

if __name__ == "__main__":
    main()
