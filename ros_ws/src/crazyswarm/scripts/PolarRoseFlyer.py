"""
Code written by Calder Leppitsch
Cooper Union ME 24

template code provided by Jason Chen
https://github.com/jasonc486
"""
#!/usr/bin/env python

import numpy as np
import numpy.linalg as LA
from pycrazyswarm import *

# Set the height of drone
Z = 0.5
# Set the numerator and denominator of the rose constant of the form r=cos((N/D)theta)
numerator = 4.0
denominator = 1.0
#Set the radius of the rose
radius = 1.0
#Give the period of the rose
period = 2*np.pi

sleeprate = 100

def runtimecalculator(N, D, R):
    K = N/D
    dtheta = 0.0001
    y = np.zeros(np.int(period*10000))
    for i in range(0,np.int(period*10000)):
        y[i] = np.sqrt((K**2)*((np.sin(K*i*dtheta))**2)+((np.cos(K*i*dtheta))**2))
    integral = np.trapz(y,dx=dtheta)
    return 2.0*R*integral

def positioncalculator(K, R, t):
    return np.array[R*np.cos(K*t)*np.cos(t), R*np.cos(K*t)*np.sin(t), 0.]

def velocitycalculator(K, R, t):
    return np.array[R*-1*(K*np.sin(K*t)*np.cos(t)+np.cos(K*t)*np.sin(t)), R*(np.cos(K*t)*np.cos(t)-K*np.sin(K*t)*np.sin(t)), 0.]

def Rose(timeHelper, flyer, N, D, R, SR, RT):
        T0 = timeHelper.time()
        startPos = flyer.initialPosition + np.array([0, 0, Z])
        K = N/D
        #Time to theta conversion factor
        CF = period/runtime
        greenlight = True
        #Start Rose Flying Routine
        while greenlight:
            time = timeHelper.time() - T0
            #Checks when to stop
            if(time >= RT):
                greenlight = False
            #Calclated desired position and velocity
            dPos = positioncalculator(K, R, CF*time)
            dVel = desiredPosition(K, R, CF*time) 
            #Send desired position to drone
            flyer.cmdPosition(dPos)
            flyer.cmdVelocity(dVel)
            #Sleep
            timeHelper.sleepForRate(SR)
        #Return to original position
        for i in range(SR*2):
            flyer.cmdPosition(startPos)
            timeHelper.sleepForRate(SR)
        
if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    allcfs.takeoff(targetHeight=Z, duration=2.0)
    flyer = allcfs.crazyflies[0]
    timeHelper.sleep(3.0)
    runtime = runtimecalculator(numerator, denominator, radius)
    Rose(timeHelper, flyer, numerator, denominator, radius, sleeprate, runtime)
    allcfs.land(targetHeight=0.06, duration=2.0)
    timeHelper.sleep(3.0)
