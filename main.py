from reachy_sdk import ReachySDK
from reachyController import ReachyController
from . import timeSeries as ts

def main():
    reachy  = ReachySDK(host='localhost')
    reachyC = ReachyController(reachy)

    reachyC.turnOn()
    reachyC.armLeft._debug_placeHandOnTable()

    b = ts.TimeSeries.loadFromCSV("test.csv")
    reachyC.playRecord(b)

if __name__ == "__main__":
    main()
