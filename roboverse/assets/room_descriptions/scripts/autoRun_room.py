#/usr/bin/env python
# Script which goes with animals_description package.
# Easy way to test planning algo (no internal DoF) on SO3 joint.

from hpp.corbaserver.ant import Robot
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver
from hpp.gepetto import Viewer, PathPlayer

robot = Robot ('ant')
robot.setJointBounds('base_joint_xyz', [-5, 5, -5, 5, -0.01, 3])
ps = ProblemSolver (robot)
cl = robot.client

r = Viewer (ps)
pp = PathPlayer (robot.client, r)
r.loadObstacleModel ("room_description","room","room")
r([1.781, 1.4, 1.3, 0, 0, 0, 1, 0, 0, 1])
