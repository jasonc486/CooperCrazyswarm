"""Takeoff-hover-land for one CF. Useful to validate hardware config."""

from pycrazyswarm import Crazyswarm
import numpy as np


TAKEOFF_DURATION = 2.5
HOVER_DURATION = 5.0
sleepRate = 10
def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[1]

    cf.takeoff(targetHeight=1.0, duration=TAKEOFF_DURATION)

    timeHelper.sleep(TAKEOFF_DURATION + HOVER_DURATION)
    keepGoing = True
    startTime = timeHelper.time()
    while keepGoing:
        time = timeHelper.time() - startTime
        currYaw = 360 / 10 * time
        cf.cmdPosition([0, 0, 1.0], yaw = currYaw)
        print(currYaw)
        if(currYaw >= 720):
            keepGoing = False
        timeHelper.sleepForRate(sleepRate)


    cf.land(targetHeight=0.04, duration=2.5)
    timeHelper.sleep(TAKEOFF_DURATION)


if __name__ == "__main__":
    main()
