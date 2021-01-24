#/usr/bin/env python

from hpp.gepetto import Viewer, PathPlayer
from hpp.corbaserver.hrp2 import Robot
from hpp.corbaserver import ProblemSolver
from hpp.corbaserver.wholebody_step.client import Client as WsClient
import time

Robot.urdfSuffix = '_capsule'
Robot.srdfSuffix= '_capsule'

robot = Robot ('hrp2_14')
robot.setJointBounds ('base_joint_xyz', [-4, 4, -4, 4, 0.6, 0.7])
ps = ProblemSolver (robot)
cl = robot.client
Viewer.withFloor = True
r = Viewer (ps)
r.loadObstacleModel ("room_description","room","room")
r.loadObstacleModel ("room_description","walls","walls")
pp = PathPlayer (cl, r)
q0 = robot.getInitialConfig ()
r (q0)

q1 = [1, 2.2, 0.64, 0.707, 0, 0, 0.707, 0.0, 0.0, 0.0, 0.0, 0.261799, 0.17453, 0.0, -0.523599, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]

q2 = [0.5, -2.0, 0.64, 0.707, 0, 0, -0.707, 0.0, 0.0, 0.0, 0.0, -1.85, 0.2, -0.7, -0.7, 0.3, 0.3, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.5, -0.2, 0.0, -0.3, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]
r(q1)

# Add constraints
wcl = WsClient ()
wcl.problem.addStaticStabilityConstraints ("balance", q0, robot.leftAnkle,
                                           robot.rightAnkle)

ps.setNumericalConstraints ("balance", ["balance/relative-com",
                                                "balance/relative-orientation",
                                                "balance/relative-position",
                                                "balance/orientation-left-foot",
                                                "balance/position-left-foot"])

res = ps.applyConstraints (q1)
if res [0]:
    q1proj = res [1]
else:
    raise RuntimeError ("Failed to apply constraint.")

res = ps.applyConstraints (q2)
if res [0]:
    q2proj = res [1]
else:
    raise RuntimeError ("Failed to apply constraint.")

cl.obstacle.loadObstacleModel('room_description','room','')
ps.setInitialConfig (q1proj); ps.addGoalConfig (q2proj)
ps.solve ()
begin=time.time()
ps.optimizePath (0)
end=time.time()
print "Solving time: "+str(end-begin)


len(ps.nodes ())
ps.pathLength(0)
ps.pathLength(1)

pp (1)


## DEBUG commands
cl.obstacle.getObstaclePosition('obstacle_base')
cl.robot.getJointOuterObjects('CHEST_JOINT1')
cl.robot.getCurrentConfig()
cl.robot.setCurrentConfig(q1)
cl.robot.collisionTest()
res = cl.robot.distancesToCollision()
cl.problem.pathLength(1)
r( cl.problem.configAtDistance(1,5) )
cl.problem.clearRoadmap ()
cl.problem.optimizePath (2)
cl.problem.directPath(q1,q2)
from numpy import *
argmin(cl.robot.distancesToCollision()[0])


r( [0.5, -2.2, 0.64, 0.7071067812, 0, 0, -0.7071067812, 0.0, 0.0, 0.0, 0.0, 1.0, 0, -1.4, -1.0, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0])
