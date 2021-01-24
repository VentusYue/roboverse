#/usr/bin/env python
# Author : Mylene Campana
# Script to plot some graphs after launching a hpp-corbaserver and solving 
# the problem. The main DIFFERENCE here is that the plot is always returned so that 
# plots can be added later in the Python interface.
# User has to call himself "plt.show()"

#import matplotlib.pyplot as plt
from __future__ import division
import numpy as np
import math

dt = 0.02 # global drawing step size
Pi = math.pi

# --------------------------------------------------------------------#

# Trajectory with double projected (on plane_theta) cones
## Parameters:
# cl: corbaserver client
# nPath: path number
# theta: theta angle made by q2-q1 (s.t. x) in xy-plane
# NconeOne: 2D direction of first cone, projected in plane_theta
# pointsConeOne: points defining the intersection of first cone with plane_theta, already expressed in plane_theta coordinates
# NconeTwo: 2D direction of second cone, projected in plane_theta
# pointsConeTwo: points defining the intersection of second cone with plane_theta, already expressed in plane_theta coordinates
# plt: matplotlib window
def parabPlotDoubleProjCones (cl, nPath, theta, NconeOne, pointsConeOne, NconeTwo, pointsConeTwo, plt):
    init = cl.problem.getInitialConfig ()
    goal = cl.problem.getGoalConfigs ()[0] # first goal
    
    # Plot whole projected path
    for t in np.arange(0., cl.problem.pathLength(nPath), dt):
        x = cl.problem.configAtParam(nPath, t)[0]
        dx = cl.problem.configAtParam(nPath, t+dt)[0]
        y = cl.problem.configAtParam(nPath, t)[1]
        dy = cl.problem.configAtParam(nPath, t+dt)[1]
        x_theta = x*math.cos(theta) + y*math.sin(theta)
        dx_theta = dx*math.cos(theta) + dy*math.sin(theta)
        plt.plot([x_theta, dx_theta], [cl.problem.configAtParam(nPath, t)[2], \
                     cl.problem.configAtParam(nPath, t+dt)[2]], 'b', linewidth=1.2)
    
    # Plot first cone (here x values are already projected on planeTheta)
    xOrigin = pointsConeOne [0]; zOrigin = pointsConeOne [1]
    xPlus = pointsConeOne [2]; zPlus = pointsConeOne [3]
    xMinus = pointsConeOne [4]; zMinus = pointsConeOne [5]
    plt.plot([xOrigin, xOrigin+xPlus],[zOrigin, zOrigin+zPlus], 'r', linewidth=1.1)
    plt.plot([xOrigin, xOrigin+xMinus],[zOrigin, zOrigin+zMinus], 'r', linewidth=1.1)
    plt.plot([xOrigin, xOrigin + NconeOne[0]],[zOrigin, zOrigin+NconeOne[1]], 'k', linewidth=1.1)
    
    # Plot second cone (here x values are already projected on planeTheta)
    xOrigin = pointsConeTwo [0]; zOrigin = pointsConeTwo [1]
    xPlus = pointsConeTwo [2]; zPlus = pointsConeTwo [3]
    xMinus = pointsConeTwo [4]; zMinus = pointsConeTwo [5]
    plt.plot([xOrigin, xOrigin+xPlus],[zOrigin, zOrigin+zPlus], 'm', linewidth=1.1)
    plt.plot([xOrigin, xOrigin+xMinus],[zOrigin, zOrigin+zMinus], 'm', linewidth=1.1)
    plt.plot([xOrigin, xOrigin + NconeTwo[0]],[zOrigin, zOrigin+NconeTwo[1]], 'k', linewidth=1.1)
    
    #plt.legend()
    #plt.axis([-2, 13, -6, 7])
    plt.xlabel('x_theta'); plt.ylabel('z')
    #plt.title('parabola planning')
    plt.grid()
    plt.plot(init[0]*math.cos(theta) + init[1]*math.sin(theta), init[2], 'go')
    plt.plot(goal[0]*math.cos(theta) + goal[1]*math.sin(theta), goal[2], 'go')
    return plt

# --------------------------------------------------------------------#

# With cones brought back to the origin
## Parameters:
# cl: corbaserver client
# nPath: path number
# theta: theta angle made by q2-q1 (s.t. x) in xy-plane
# NconeOne: 2D direction of first cone, projected in plane_theta
# pointsConeOne: points defining the intersection of first cone with plane_theta, already expressed in plane_theta coordinates
# angles: limit angles from constraints [alpha_min, alpha_max, alpha_lim+, alpha_lim-, alpha_inf, alpha_sup, alpha_inf4]
# i: index in each angles vector (typically, number of occurence in log file)
# ampl: cone amplitude
# plt: matplotlib window
def parabPlotOriginCones (cl, nPath, theta, NconeOne, pointsConeOne, angles, i, ampl, plt):
    init = cl.problem.getInitialConfig ()
    goal = cl.problem.getGoalConfigs ()[0] # first goal
    
    # Plot first cone (here x values are already projected on planeTheta)
    xOrigin = pointsConeOne [0]; zOrigin = pointsConeOne [1]
    xPlus = pointsConeOne [2]; zPlus = pointsConeOne [3]
    xMinus = pointsConeOne [4]; zMinus = pointsConeOne [5]
    plt.plot([xOrigin, xOrigin+xPlus],[zOrigin, zOrigin+zPlus], 'g', linewidth=1.1)
    plt.plot([xOrigin, xOrigin+xMinus],[zOrigin, zOrigin+zMinus], 'g', linewidth=1.1)
    #plt.plot([xOrigin, xOrigin + NconeOne[0]],[zOrigin, zOrigin+NconeOne[1]], 'k', linewidth=1.1)
    
    # Plot each 'constraint-cone'
    alpha_min = angles [0][i]
    alpha_max = angles [1][i]
    plt.plot([xOrigin, xOrigin + ampl*math.cos(alpha_min)],[zOrigin, zOrigin + ampl*math.sin(alpha_min)], 'c', linewidth=1.1)
    plt.plot([xOrigin, xOrigin + ampl*math.cos(alpha_max)],[zOrigin, zOrigin + ampl*math.sin(alpha_max)], 'c', linewidth=1.1)
    
    alpha_minus = angles [2][i] # corresponds to alpha_lim-
    alpha_plus = angles [3][i] # corresponds to alpha_lim+
    plt.plot([xOrigin, xOrigin + ampl*math.cos(alpha_minus)],[zOrigin, zOrigin + ampl*math.sin(alpha_minus)], 'k', linewidth=1.1)
    plt.plot([xOrigin, xOrigin + ampl*math.cos(alpha_plus)],[zOrigin, zOrigin + ampl*math.sin(alpha_plus)], 'k', linewidth=1.1)
    
    alpha_inf = angles [4][i] # corresponds to alpha_inf
    alpha_sup = angles [5][i] # corresponds to alpha_sup
    plt.plot([xOrigin, xOrigin + ampl*math.cos(alpha_inf)],[zOrigin, zOrigin + ampl*math.sin(alpha_inf)], 'm', linewidth=1.1)
    plt.plot([xOrigin, xOrigin + ampl*math.cos(alpha_sup)],[zOrigin, zOrigin + ampl*math.sin(alpha_sup)], 'm', linewidth=1.1)
    
    alpha_inf4 = angles [6][i]
    plt.plot([xOrigin, xOrigin + ampl*math.cos(alpha_inf4)],[zOrigin, zOrigin + ampl*math.sin(alpha_inf4)], 'r', linewidth=1.1)
    plt.plot([xOrigin, xOrigin + ampl*math.cos(Pi/2)],[zOrigin, zOrigin + ampl*math.sin(Pi/2)], 'r', linewidth=1.1)
    
    # Plot first quarter of the whole projected path
    for t in np.arange(0., cl.problem.pathLength(nPath), dt):
        x = cl.problem.configAtParam(nPath, t)[0]
        dx = cl.problem.configAtParam(nPath, t+dt)[0]
        y = cl.problem.configAtParam(nPath, t)[1]
        dy = cl.problem.configAtParam(nPath, t+dt)[1]
        x_theta = x*math.cos(theta) + y*math.sin(theta)
        dx_theta = dx*math.cos(theta) + dy*math.sin(theta)
        plt.plot([x_theta, dx_theta], [cl.problem.configAtParam(nPath, t)[2], \
                     cl.problem.configAtParam(nPath, t+dt)[2]], 'b--', linewidth=1.2)
    
    #plt.legend()
    #plt.axis([-2, 13, -6, 7])
    plt.xlabel('x'); plt.ylabel('y')
    #plt.title('parabola planning')
    plt.grid()
    plt.plot(init[0]*math.cos(theta) + init[1]*math.sin(theta), init[2], 'go')
    plt.plot(goal[0]*math.cos(theta) + goal[1]*math.sin(theta), goal[2], 'go')
    return plt

# --------------------------------------------------------------------#

# Plot 2D nodes (from parseLog) with given color and text.
# For example, nodeName = r'qCol'    and   nodeColor = 'bo'
def addNodePlot (nodeList, nodeColor, nodeName, plt):
    i = 0
    for n in nodeList :
        plt.plot(n[0], n[1], nodeColor)
        #plt.text(n[0]+.02, n[1], nodeName+'%i' %(i), fontsize=8)
        i = i+1
    return plt


# --------------------------------------------------------------------#

# Plot a parsed parabola
def plotParsedParab (parab, color, lw, plt):
    for i in np.arange(0, len(parab)-1, 1):
        plt.plot([parab[i][0], parab[i+1][0]],[parab[i][1], parab[i+1][1]], color, linewidth=lw)
    return plt

# --------------------------------------------------------------------#

# Plot 2D rectangles for parabola environment
def plotRectangles_parab (plt):
    obstacles = [[-4,6],[-4,2],[-2,0],[0,0],[4,-4],[4,-3],[6,-3],[8,-1],[8,2],[10,2],[10,-6],[13,-6],[13,6]];
    obstacles = np.array (obstacles)
    lw = 5.5 # linewidth
    obstColor = 'r'
    for i in np.arange(0, len(obstacles)-1, 1):
        plt.plot([obstacles[i][0], obstacles[i+1][0]], [obstacles[i][1], obstacles[i+1][1]], obstColor, linewidth=lw)
    return plt

# --------------------------------------------------------------------#

# Plot 2D rectangles for parabola harder environment
def plotHardRectangles_parab (plt):
    lw = 2 # linewidth
    obstColor = 'r'
    obstacles = [[-4,6],[-4,2],[-2,0],[0,0],[4,-4],[4,-3],[6,-3],[8,-1],[8,2],[10,2],[10,-6],[13,-6],[13,-4],[11,-4],[11,-3],[13,-3],[13,6]];
    obstacles = np.array (obstacles)
    
    obstacles1 = [[10,0],[11,0],[11,1],[10,1],[10,0]] # additional obst
    obstacles1 = np.array (obstacles1)
    
    obstacles2 = [[0,2],[1,2],[1,3],[0,3],[0,2]] # additional obst
    obstacles2 = np.array (obstacles2)
    
    obstacles3 = [[4,3],[5,3],[5,4],[4,4],[4,3]] # additional obst
    obstacles3 = np.array (obstacles3)
    
    for i in np.arange(0, len(obstacles)-1, 1):
        plt.plot([obstacles[i][0], obstacles[i+1][0]], [obstacles[i][1], obstacles[i+1][1]], obstColor, linewidth=lw)
    for i in np.arange(0, len(obstacles1)-1, 1):
        plt.plot([obstacles1[i][0], obstacles1[i+1][0]], [obstacles1[i][1], obstacles1[i+1][1]], obstColor, linewidth=lw)
    for i in np.arange(0, len(obstacles2)-1, 1):
        plt.plot([obstacles2[i][0], obstacles2[i+1][0]], [obstacles2[i][1], obstacles2[i+1][1]], obstColor, linewidth=lw)
    for i in np.arange(0, len(obstacles3)-1, 1):
        plt.plot([obstacles3[i][0], obstacles3[i+1][0]], [obstacles3[i][1], obstacles3[i+1][1]], obstColor, linewidth=lw)
    return plt

