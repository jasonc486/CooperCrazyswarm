#!/usr/bin/env python

import asyncio
import numpy as np
from pycrazyswarm import *

# Set the hovering height of all drones
Z = 0.5
# Determines the number of samples taken while flying in circle
sleepRate = 30
# Number of complete orbits "Earth" will complete
Trials = 2
# Radius of orbits in meters
earthOrbitRadius = 1.0
moonOrbitRadius = 0.25
# Orbital periods in seconds
earthOrbitPeriod = 4.0
moonOrbitPeriod = 2.0

kPosition = 1

def positionChecker(initialPos, currentPos):
    return (((initialPos[0] - 0.01) < currentPos[0]) and (currentPos[0] < (initialPos[0] + 0.01))) 

async def goCircle(timeHelper, cf, totalTime, radius, kPosition):
        startTime = timeHelper.time()
        pos = cf.position()
        tempPos = cf.position()
        trialCounter = 0
        startPos = cf.initialPosition + np.array([0, 0, Z])
        center_circle = startPos - np.array([radius, 0, 0])
        print(pos)
        keepGoing = True
        while keepGoing:
            if (positionChecker(pos, cf.position()) and 
                    not positionChecker(pos, tempPos)):
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
            await asyncio.sleep(0.01)

async def goCircle2(timeHelper, cf, totalTime, radius, kPosition):
        startTime = timeHelper.time()
        pos = cf.position()
        tempPos = cf.position()
        trialCounter = 0
        startPos = cf.initialPosition + np.array([0, 0, Z])
        center_circle = startPos - np.array([radius, 0, 0])
        print(pos)
        keepGoing = True
        while keepGoing:
            if (positionChecker(pos, cf.position()) and 
                    not positionChecker(pos, tempPos)):
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

# if __name__ == "__main__":
async def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    cf1 = allcfs.crazyflies[0]
    cf2 = allcfs.crazyflies[1]
    timeHelper.sleep(2 + Z)
    f1 = loop.create_task(goCircle(timeHelper, cf1, earthOrbitPeriod, earthOrbitRadius, kPosition))
    f2 = loop.create_task(goCircle2(timeHelper, cf2, earthOrbitPeriod, earthOrbitRadius, kPosition))
    await asyncio.wait([f1, f2])
swarm = Crazyswarm()
timeHelper = swarm.timeHelper
allcfs = swarm.allcfs
loop = asyncio.get_event_loop()
loop.run_until_complete(main())
loop.close()
allcfs.land(targetHeight=0.06, duration=2.0)
timeHelper.sleep(3.0)
