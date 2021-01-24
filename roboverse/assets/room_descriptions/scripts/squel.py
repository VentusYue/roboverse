#/usr/bin/env python

from hpp.gepetto import Viewer, PathPlayer
from hpp.corbaserver.squel_robot import Robot
from hpp.corbaserver import ProblemSolver
from hpp.corbaserver.wholebody_step.client import Client as WsClient
import time

robot = Robot ('squel_robot')
robot.setJointBounds ('base_joint_xyz', [-3, 3, -3, 3, 0, 1])
ps = ProblemSolver (robot)
cl = robot.client
Viewer.withFloor = True
r = Viewer (ps)
#r.loadObstacleModel ("room_description","room","room")
#r.loadObstacleModel ("room_description","walls","walls")
#pp = PathPlayer (cl, r)

#r.loadObstacleModel ("room_description","squel","squel")

# Add constraints
"""
wcl = WsClient ()
wcl.problem.addStaticStabilityConstraints ("balance", q0, robot.leftAnkle,
                                           robot.rightAnkle)

ps.setNumericalConstraints ("balance", ["balance/relative-com",
                                                "balance/relative-orientation",
                                                "balance/relative-position",
                                                "balance/orientation-left-foot",
                                                "balance/position-left-foot"])
"""

q1 = [0.0, 0.0, 1.2, 0, 0, 1.57, 0.0, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0.0, 0.0, 0, 0, 0, -0.174532, 0, 0, 0, 0, 0.0, 0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, 0.0, 0.0, 0, 0, 0, 0.0, 0.0, 0.0, 0, 0, 0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
r(q1)

"""
res = ps.applyConstraints (q1)
if res [0]:
    q1proj = res [1]
else:
    raise RuntimeError ("Failed to apply constraint.")
"""

q2 = [0.0, 0.0, 1.2, 0, 0, 1.57, 0.0, 0.0, 0.0, 0.0, 0, 0, 0, 0, 0.0, 0.0, 0, 0, 0, -0.174532, 0, 0, 0, 0, 0.1, 0, 0.0, 0.0, 0, 0, 0, 0, 0, 0, 0.0, 0.0, 0, 0, 0, 0.0, 0.0, 0.0, 0, 0, 0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
r(q2)

"""
res = ps.applyConstraints (q2)
if res [0]:
    q2proj = res [1]
else:
    raise RuntimeError ("Failed to apply constraint.")

ps.setInitialConfig (q1proj); ps.addGoalConfig (q2proj)
"""

ps.setInitialConfig (q1); ps.addGoalConfig (q2)

#cl.obstacle.loadObstacleModel('room_description','room','')
#cl.obstacle.loadObstacleModel('room_description','walls','')

#ps.selectPathPlanner ("VisibilityPrmPlanner") 
begin=time.time()
ps.solve ()
end=time.time()
print "Solving time: "+str(end-begin)

ps.addPathOptimizer("GradientBased")
begin=time.time()
ps.optimizePath (0)
end=time.time()
print "Optim time: "+str(end-begin)

cl.problem.getIterationNumber ()
ps.pathLength(0)
ps.pathLength(1)

ps.optimizePath (1)  # first use of active comp 1.601s  19iter
ps.pathLength(2)

len(ps.getWaypoints (0))

pp (1)


## Video recording
r.startCapture ("capture","png")
pp(1)
r.stopCapture ()
#ffmpeg -r 50 -i capture_0_%d.png -r 25 -vcodec libx264 video.mp4

## DEBUG commands
cl.obstacle.getObstaclePosition('obstacle_base')
cl.robot.getJointOuterObjects('CHEST_JOINT1')
cl.robot.getCurrentConfig()
robot.isConfigValid(q1)
res = cl.robot.distancesToCollision()
cl.problem.pathLength(1)
r(cl.problem.configAtParam(1,5))
cl.problem.clearRoadmap ()
cl.problem.optimizePath (2)
cl.problem.directPath(q1,q2)
from numpy import *
argmin(cl.robot.distancesToCollision()[0])

