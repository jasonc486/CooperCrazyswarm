#!/usr/bin/env python

import numpy as np

from pycrazyswarm import * 
import uav_trajectory

print(dir(Crazyswarm()))

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    print("Created swarm and timeHelper")
    #allcfs = swarm.allcfs
    cf2 = swarm.allcfs.crazyflies[1]
    traj1 = uav_trajectory.Trajectory()
    traj1.loadcsv("figure8.csv")
    print("Loaded trajectory csv file")
    TRIALS = 1
    TIMESCALE = 1.0
    for i in range(TRIALS):
        #for cf in allcfs.crazyflies:
        #    cf.uploadTrajectory(0, 0, traj1)
        cf2.uploadTrajectory(0, 0, traj1)
        print("Uploaded trajectory to crazyfly")
        #allcfs.takeoff(targetHeight=1.0, duration=2.0)
        print("Takeoff start")
        cf2.takeoff(targetHeight=1.0, duration=2.0)
        print("Takeoff completed")
        timeHelper.sleep(2.5)
        #for cf in allcfs.crazyflies:
        #    pos = np.array(cf.initialPosition) + np.array([0, 0, 1.0])
        #    cf.goTo(pos, 0, 2.0)

        pos = np.array(cf2.initialPosition) + np.array([0, 0, 1.0])
        cf2.goTo(pos, 0, 2.0)
        print("Go to Position")
        timeHelper.sleep(2.5)

        #allcfs.startTrajectory(0, timescale=TIMESCALE)
        print("Started trajectory")
        cf2.startTrajectory(0, timescale=TIMESCALE)
        timeHelper.sleep(traj1.duration * TIMESCALE + 2.0)
        print("Do trajectory again but reversed")        
        #allcfs.startTrajectory(0, timescale=TIMESCALE, reverse=True)
        cf2.startTrajectory(0, timescale=TIMESCALE, reverse=True)
        timeHelper.sleep(traj1.duration * TIMESCALE + 2.0)
        print("Start landing")
        #allcfs.land(targetHeight=0.06, duration=2.0)
        cf2.land(targetHeight=0.06, duration=2.0)
        timeHelper.sleep(3.0)
