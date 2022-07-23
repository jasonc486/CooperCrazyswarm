#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *


Z = 0.5
sleepRate = 30
Trials = 5
i = 0

def positionChecker(initialPos, currentPos):
    return (((initialPos[0] - 0.01) < currentPos[0]) and (currentPos[0] < (initialPos[0] + 0.01))) 

def goCircle(timeHelper, cf, totalTime, radius, kPosition):
        startTime = timeHelper.time()
        pos = cf.position()
        tempPos = cf.position()
        tempBool = True
        trialCounter = 0
        startPos = cf.initialPosition + np.array([0, 0, Z])
        center_circle = startPos - np.array([radius, 0, 0])
        print(pos)
        keepGoing = True
        while keepGoing:
            if positionChecker(pos, cf.position()) and not positionChecker(pos, tempPos):
                print("This works")
                trialCounter += 1
                if trialCounter == Trials:
                    keepGoing = False
            tempPos = cf.position()
            time = timeHelper.time() - startTime
            omega = 2 * np.pi / totalTime
            vx = -radius * omega * np.sin(omega * time)  
            vy = radius * omega * np.cos(omega * time)
            desiredPos = center_circle + radius * np.array(
                [np.cos(omega * time), np.sin(omega * time), 0])
            errorX = desiredPos - cf.position() 
            cf.cmdVelocityWorld(np.array([vx, vy, 0] +
                                         kPosition * errorX),
                                yawRate=0)
            timeHelper.sleepForRate(sleepRate)
            #print(cf.position())


if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(2 + Z)
    goCircle(timeHelper, allcfs.crazyflies[0], totalTime=4, radius=1, kPosition=1)
    allcfs.land(targetHeight=0.06, duration=2.0)
    timeHelper.sleep(3.0)
