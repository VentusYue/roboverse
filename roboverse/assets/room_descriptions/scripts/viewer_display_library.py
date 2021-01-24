#/usr/bin/env python
# Script which goes with animals_description package.
# Functions library to display graphic things (trajectory, cone, frame...) on the Gepetto-viewer

from __future__ import division
import numpy as np
import math
#from hpp.gepetto import Viewer, PathPlayer

Pi = math.pi

# --------------------------------------------------------------------#

# Normalize the dir part of the configuration (dir = direction of cone)
## Parameters:
# q: given configuration
# robot: the robot
def normalizeDir (q, robot):
    q_out = q[::] # copy
    index = robot.getConfigSize () - 3
    N_norm = math.sqrt (q [index]**2 + q [index+1]**2 + q [index+2]**2)
    q_out [index] = q [index]/N_norm
    q_out [index+1] = q [index+1]/N_norm
    q_out [index+2] = q [index+2]/N_norm
    return q_out;

# --------------------------------------------------------------------#

## Plot whole path in viewer (in blue) ##
## Parameters:
# cl: corbaserver client
# nPath: path number
# r: viewer server
# lineNamePrefix: string prefix used for line name
# dt: step time
def plotPath (cl, nPath, r, lineNamePrefix, dt):
    for t in np.arange(0., cl.problem.pathLength(nPath), dt):
        lineName = lineNamePrefix+str(t)
        r.client.gui.addLine(lineName,[cl.problem.configAtParam(nPath, t)[0],cl.problem.configAtParam(nPath, t)[1],cl.problem.configAtParam(nPath, t)[2]],[cl.problem.configAtParam(nPath, t+dt)[0],cl.problem.configAtParam(nPath, t+dt)[1],cl.problem.configAtParam(nPath, t+dt)[2]],[0,0.3,1,1])
        r.client.gui.addToGroup (lineName, r.sceneName)

# --------------------------------------------------------------------#

## Plot global frame (in white, x darker) ##
## Parameters:
# r: viewer server
# lineNameSuffix: string suffix used for line name, allowing to plot several frames
# framePosition: the [x, y, z] absolute position of the frame
# ampl: amplitude of the frame axis
def plotFrame (r, lineNameSuffix, framePosition, ampl):
    x = framePosition [0]; y = framePosition [1]; z = framePosition [2];
    r.client.gui.addLine("frame1"+lineNameSuffix,[x,y,z], [x+ampl,y,z],[1,0,0,1])
    r.client.gui.addToGroup ("frame1"+lineNameSuffix, r.sceneName)
    r.client.gui.addLine("frame2"+lineNameSuffix,[x,y,z], [x,y+ampl,z],[0,1,0,1])
    r.client.gui.addToGroup ("frame2"+lineNameSuffix, r.sceneName)
    r.client.gui.addLine("frame3"+lineNameSuffix,[x,y,z], [x,y,z+ampl],[0,0,1,1])
    r.client.gui.addToGroup ("frame3"+lineNameSuffix, r.sceneName)

# --------------------------------------------------------------------#

## Plot VERTICAL cone at each waypoint of the path ##
## Parameters:
# cl: corbaserver client
# nPath: path number
# r: viewer server
# mu: cone coefficient of friction
# ampl: cone amplitude
# lineNamePrefix: string prefix used for line name
def plotVerticalConeWaypoints (cl, nPath, r, mu, ampl, lineNamePrefix):
    wp = cl.problem.getWaypoints (nPath)
    for i in np.arange(0, len(wp), 1):
        plotVerticalCone (wp[i], r, mu, ampl, lineNamePrefix)

# --------------------------------------------------------------------#

## Plot cone at each waypoint of the path ##
## Parameters:
# cl: corbaserver client
# nPath: path number
# r: viewer server
# mu: cone coefficient of friction
# ampl: cone amplitude
# lineNamePrefix: string prefix used for line name
def plotConeWaypoints (cl, nPath, r, mu, ampl, lineNamePrefix):
    wp = cl.problem.getWaypoints (nPath)
    for i in np.arange(0, len(wp), 1):
        plotCone (wp[i], cl, r, mu, ampl, lineNamePrefix+str(i))

# --------------------------------------------------------------------#

## Plot VERTICAL cone ##
## Parameters:
# q: configuration of cone (position, orientation)
# r: viewer server
# mu = cone coefficient of friction
# ampl: cone amplitude
# lineNamePrefix: string prefix used for line name
def plotVerticalCone (q, r, mu, ampl, lineNamePrefix):
    phi = math.atan(mu)
    x = q[0]
    y = q[1]
    z = q[2]
    for angle in np.arange(0, 2*math.pi, 0.2):
        x_theta = ampl*math.cos(angle)
        y_theta = ampl*math.sin(angle)
        z_theta = ampl/math.sin(phi)
        lineName = lineNamePrefix+str(angle)
        r.client.gui.addLine(lineName,[x,y,z], [x+x_theta,y+y_theta,z+z_theta],[0,1,0.3,1])
        r.client.gui.addToGroup (lineName, r.sceneName)

# --------------------------------------------------------------------#

## Plot cone ##
## Parameters:
# cl: corbaserver client
# q: configuration of cone (position, orientation)
# r: viewer server
# mu: cone coefficient of friction
# ampl: cone amplitude
# lineNamePrefix: string prefix used for line name
def plotCone (q, cl, r, mu, ampl, lineNamePrefix):
    index = cl.robot.getConfigSize () - 3
    phi = math.atan(mu)
    x0 = q[0]
    y0 = q[1]
    z0 = q[2]
    U = q[index]
    V = q[index+1]
    W = q[index+2]

    for angle in np.arange(0, 2*math.pi, 0.05):
        x = ampl*math.cos(angle)
        y = ampl*math.sin(angle)
        discrCone = (U**2 + V**2 + W**2)*(U**2*mu**2*x**2 - U**2*y**2 + 2*U*V*mu**2*x*y + 2*U*V*x*y + V**2*mu**2*y**2 - V**2*x**2 + W**2*mu**2*x**2 + W**2*mu**2*y**2)
        if discrCone >= 0:
            z1 = coneFunctionOne (x, y, U, V, W, mu, discrCone)
            z2 = coneFunctionTwo (x, y, U, V, W, mu, discrCone)
            lineName = lineNamePrefix+str(angle)
            r.client.gui.addLine(lineName,[x0,y0,z0], [x0+x,y0+y,z0+z1],[0,1,0.3,1])
            r.client.gui.addToGroup (lineName, r.sceneName)
            r.client.gui.addLine("Second"+lineName,[x0,y0,z0], [x0+x,y0+y,z0+z2],[0,0.6,0.6,1])
            r.client.gui.addToGroup ("Second"+lineName, r.sceneName)


# --------------------------------------------------------------------#

## Cone function 1 ##
# [U, V, W] = cone direction
# discrCone = discriminant of cone equation
def coneFunctionOne (x, y, U, V, W, mu, discrCone):
    z = (math.sqrt(discrCone) + U*W*x + V*W*y + U*W*mu**2*x + V*W*mu**2*y)/(U**2 + V**2 - W**2*mu**2)
    return z


## Cone function 2 ##
# [U, V, W] = cone direction
# discrCone = discriminant of cone equation
def coneFunctionTwo (x, y, U, V, W, mu, discrCone):
    z = (-math.sqrt(discrCone) + U*W*x + V*W*y + U*W*mu**2*x + V*W*mu**2*y)/(U**2 + V**2 - W**2*mu**2)
    return z

# --------------------------------------------------------------------#

## Plot straight line ##
# Uses: plot cone direction or cone - plane_theta intersection
## Parameters:
# vector: direction vector
# q: current configuration (origin of straight line)
# r: viewer server
# lineNamePrefix: string prefix used for line name
def plotStraightLine (vector, q, r, lineNamePrefix):
    x0 = q[0]
    y0 = q[1]
    z0 = q[2]
    x = vector[0]
    y = vector[1]
    z = vector[2]
    lineName=lineNamePrefix+"straight"
    r.client.gui.addLine(lineName,[x0,y0,z0], [x0+x,y0+y,z0+z],[1,0.3,0.3,1])
    r.client.gui.addToGroup (lineName, r.sceneName)

# --------------------------------------------------------------------#

## Plot plane theta ##
# (with two triangles, last pos not taken in addSquareFace)
## Parameters:
# q1: initial configuration
# q2: final configuration
# r : viewer server
# lineNamePrefix: string prefix used for line name
def plotThetaPlane (q1, q2, r, lineNamePrefix):
    pos1 = [q1 [0], q1 [1], q1 [2]+2]
    pos2 = [q2 [0], q2 [1], q2 [2]-2]
    pos3 = [q1 [0], q1 [1], q1 [2]-2]
    pos4 = [q2 [0], q2 [1], q2 [2]+2]
    r.client.gui.addSquareFace (lineNamePrefix, pos1, pos2, pos4, pos4, [0.7,0.2,0.2,0.5])
    r.client.gui.addToGroup (lineNamePrefix, r.sceneName)
    lineNamePrefix_bis = lineNamePrefix+"bis"
    r.client.gui.addSquareFace (lineNamePrefix_bis, pos1, pos3, pos2, pos2, [0.7,0.2,0.2,0.5])
    r.client.gui.addToGroup (lineNamePrefix_bis, r.sceneName)

# --------------------------------------------------------------------#

## Shoot random config, normalize 'dir' part and plot cone ##
## Parameters:
# cl: corbaserver client
# r: viewer server
# mu: cone coefficient of friction
# ampl: cone amplitude
# lineNamePrefix: string prefix used for line name
def shootNormPlot (cl, r, mu, ampl, lineNamePrefix):
    q = cl.robot.shootRandomConfig ()
    print q
    q = normalizeDir (q, cl.robot)
    plotCone (q, cl, r, mu, ampl, lineNamePrefix)
    index = cl.robot.getConfigSize () - 3
    plotStraightLine ([q [index],q [index+1],q [index+2]], q, r, lineNamePrefix+"normale")
    return q

