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
earthRotationalPeriod = 20.0
moonRotationalPeriod = 20.0
sunRotationalPeriod = 40.0
# Ellipse axis parameters
ellipseA = 1.0
ellipseB = 1.0

kPositionE = 1
kPositionM = 1

def desiredPosition(center, radius, omega, time):
    return center + radius * np.array([ellipseA * np.cos(omega * time), ellipseB * np.sin(omega * time), 0])

def degreeCalculator(rotationalPeriod, time):
    return time * 360 / rotationalPeriod

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
            #keeps the previous position to tell if has returned
            tempPos = cfEarth.position()
            time = timeHelper.time() - startTime
            #Calculate celestial body rotation in DEGREES
            earthSpinPos = degreeCalculator(earthRotationalPeriod, time) 
            moonSpinPos = degreeCalculator(moonRotationalPeriod, time) 
            sunSpinPos = degreeCalculator(sunRotationalPeriod, time) 
            #Calculate angle
            omegaEarth = 2 * np.pi / totalTime
            omegaMoon = 2 * np.pi / moonOrbitPeriod
            #Checks when to stop
            if(omegaEarth * time >= Trials * 2 * np.pi):
                keepGoing = False
            #Calclated desired position
            desiredPosEarth = desiredPosition(center_circle, radius, omegaEarth, time)
            desiredPosMoon = desiredPosition(desiredPosEarth, moonOrbitRadius, omegaMoon, time) 
            #Send desired position to drone
            cfEarth.cmdPosition(desiredPosEarth, yaw = earthSpinPos)
            cfMoon.cmdPosition(desiredPosMoon, yaw = moonSpinPos)
            cfSun.cmdPosition(startPosS, yaw = sunSpinPos)
#            print(sunSpinPos, earthSpinPos, moonSpinPos) 
            #print(cfSun.yaw(), cfEarth.yaw(), cfMoon.yaw())
            timeHelper.sleepForRate(sleepRate)
        #Return to original positions
        for i in range(sleepRate*2):
            cfEarth.cmdPosition(startPosE, yaw=0)
            cfMoon.cmdPosition(startPosM, yaw=0)
            cfSun.cmdPosition(startPosS, yaw=0)
            timeHelper.sleepForRate(sleepRate)
        #timeHelper.sleep(2)
        
if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    cf1 = allcfs.crazyflies[0]
    cf2 = allcfs.crazyflies[1]
    cf3 = allcfs.crazyflies[2]
    timeHelper.sleep(3 + Z)
    goCircle(timeHelper, cf2, cf1, cf3, earthOrbitPeriod, earthOrbitRadius, kPositionE)
    allcfs.land(targetHeight=0.06, duration=2.0)
    timeHelper.sleep(3.0)
