#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *

# Set the hovering height of all drones
Z = 0.5
# Determines the number of samples taken while flying in circle
sleepRate = 100
# Number of complete orbits "Earth" will complete
Trials = 2
# Radius of orbits in meters
earthOrbitRadius = 1.0
moonOrbitRadius = 0.5
# Orbital periods in seconds
earthOrbitPeriod = 20.0
moonOrbitPeriod = 4.0
# Rotational periods in seconds
earthRotationalPeriod = 10.0
moonRotationalPeriod = 10.0
sunRotationalPeriod = 40.0
# Ellipse axis parameters
ellipseA = 1.0
ellipseB = 1.0

kPositionE = 1
kPositionM = 1
def positionChecker(initialPos, currentPos):
    return (((initialPos[0] - 0.01) < currentPos[0]) and
            (currentPos[0] < (initialPos[0] + 0.01))) 

def desiredPosition(center, radius, omega, time):
    return center + radius * np.array([ellipseA * np.cos(omega * time), ellipseB * np.sin(omega * time), 0])

def goCircle(timeHelper, cfEarth, cfSun, cfMoon, totalTime, radius, kPosition):
        startTime = timeHelper.time()
        pos = cfEarth.position()
        tempPos = cfEarth.position()
        trialCounter = 0
        startPosE = cfEarth.initialPosition + np.array([0, 0, Z])
        startPosM = cfMoon.initialPosition + np.array([0, 0, Z])
        startPosS = cfSun.initialPosition + np.array([0, 0, Z])
        center_circle = startPosE - np.array([radius, 0, 0])
        #print(cfEarth.initialPosition)
        keepGoing = True
        while keepGoing:
            earthPosition = cfEarth.position()
            #Calculate yaw rate for Earth rotation
            earthSpinRate = 2 * np.pi / earthRotationalPeriod
            #keeps the previous position to tell if has returned
            tempPos = cfEarth.position()
            time = timeHelper.time() - startTime
            #Calculate celestial body rotation in radians
            earthSpinPos = 2 * np.pi / earthRotationalPeriod * time
            moonSpinPos = 2 * np.pi / moonRotationalPeriod * time
            sunSpinPos = 2 * np.pi / sunRotationalPeriod * time
            #Calculate angle
            omegaEarth = 2 * np.pi / totalTime
            omegaMoon = 2 * np.pi / moonOrbitPeriod
            #Checks when to stop
            if(omegaEarth * time >= Trials * 2 * np.pi):
                keepGoing = False
                break
            #Calclated desired position
            desiredPosEarth = desiredPosition(center_circle, radius, omegaEarth, time)
            desiredPosMoon = (desiredPosEarth, moonOrbitRadius, omegaMoon, time) 
            #Send desired position to drone
            cfEarth.cmdPosition(desiredPosEarth, yaw = 0)
            cfMoon.cmdPosition(desiredPosMoon, yaw = 0)
            cfSun.cmdPosition(startPosS, yaw = 0)
#            print(sunSpinPos, earthSpinPos, moonSpinPos) 
            #print(cfSun.yaw(), cfEarth.yaw(), cfMoon.yaw())
            timeHelper.sleepForRate(sleepRate)
        #Return to original positions
        cfEarth.cmdPosition(startPosE, yaw=0)
        cfMoon.cmdPosition(startPosM, yaw=0)
        timeHelper.sleep(2.0)
        
if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    cf1 = allcfs.crazyflies[0]
    cf2 = allcfs.crazyflies[1]
    cf3 = allcfs.crazyflies[2]
    timeHelper.sleep(3 + Z)
    goCircle(timeHelper, cf1, cf2, cf3, earthOrbitPeriod,
             earthOrbitRadius, kPositionE)
    allcfs.land(targetHeight=0.06, duration=2.0)
    timeHelper.sleep(3.0)
