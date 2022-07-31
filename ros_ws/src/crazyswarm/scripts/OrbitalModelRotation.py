#!/usr/bin/env python

#import asyncio
import numpy as np
from pycrazyswarm import *

# Set the hovering height of all drones
Z = 0.5
# Determines the number of samples taken while flying in circle
sleepRate = 360
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
moonRotationalPeriod = 5.0
# Ellipse axis parameters
ellipseA = 1.0
ellipseB = 1.0

kPositionE = 1
kPositionM = 1
def positionChecker(initialPos, currentPos):
    return (((initialPos[0] - 0.01) < currentPos[0]) and
            (currentPos[0] < (initialPos[0] + 0.01))) 

def desiredVelocity(radius, omega, time):
    vx = -radius * omega * np.sin(omega * time)
    vy = radius * omega * np.cos(omega * time)
    return np.array([vx, vy, 0])

def desiredPosition(center, radius, omega, time):
    return center + radius * np.array([ellipseA * np.cos(omega * time), ellipseB * np.sin(omega * time), 0])

def goCircle(timeHelper, cfEarth, cfMoon, totalTime, radius, kPosition):
        startTime = timeHelper.time()
        pos = cfEarth.position()
        tempPos = cfEarth.position()
        trialCounter = 0
        startPosE = cfEarth.initialPosition + np.array([0, 0, Z])
        startPosM = cfMoon.initialPosition + np.array([0, 0, Z])
        center_circle = startPosE - np.array([radius, 0, 0])
        print(cfEarth.initialPosition)
        keepGoing = True
        while keepGoing:
            earthPosition = cfEarth.position()
            #Checks the number of cycles has gone through and stops the loop
            if (positionChecker(pos, earthPosition) and 
                    not positionChecker(pos, tempPos)):
                print("This works")
                trialCounter += 1
                if trialCounter == Trials:
                    keepGoing = False
            #Calculate yaw rate for Earth rotation
            earthSpinRate = 2 * np.pi / earthRotationalPeriod
            #keeps the previous position to tell if has returned
            tempPos = cfEarth.position()
            time = timeHelper.time() - startTime
            #Calculate the yaw for the Moon's rotation
            earthSpinPos = 2 * np.pi / earthRotationalPeriod * time
            moonSpinPos = 2 * np.pi / moonRotationalPeriod * time
            #Calculate angle
            omegaEarth = 2 * np.pi / totalTime
            omegaMoon = 2 * np.pi / moonOrbitPeriod
            #Calculate desired velocity for Earth
            desiredVelEarth = desiredVelocity(radius, omegaEarth, time)
            #Calclated desired position
            desiredPosEarth = desiredPosition(center_circle, radius, omegaEarth, time)
            desiredPosMoon = desiredPosition(cfEarth.position(), moonOrbitRadius, omegaMoon, time) 
            #Find the error
            errorXEarth = desiredPosEarth - cfEarth.position() 
            #Send desired velocity to drone
#            cfEarth.cmdVelocityWorld(((desiredVelEarth) + kPosition * errorXEarth), yawRate = earthSpinRate)
            #Send desired position to drone
            cfEarth.cmdPosition(desiredPosEarth, yaw = earthSpinPos)
            cfMoon.cmdPosition(desiredPosMoon, yaw = moonSpinPos)
            timeHelper.sleepForRate(sleepRate)
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
    timeHelper.sleep(2 + Z)
    goCircle(timeHelper, cf1, cf3, earthOrbitPeriod,
             earthOrbitRadius, kPositionE)
    allcfs.land(targetHeight=0.06, duration=2.0)
    timeHelper.sleep(3.0)
