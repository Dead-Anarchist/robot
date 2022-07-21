#!/usr/bin/env python3

import custom_global_planner_function as cgpf
import math 
import numpy as np
import obstacle_coordinate
from matplotlib import pyplot
from matplotlib.patches import *

def test_trajectory(x1, y1, ori1,
                    x2, y2, ori2,
                    costmap, costmap_resolution,
                    costmap_origin_x, costmap_origin_y,
                    should_be_successful, eps = 0.1):
    # trying to trace a trajectory
    success, traj = cgpf.make_trajectory(x1, y1, ori1,
                    x2, y2, ori2,
                    costmap, costmap_resolution,
                    costmap_origin_x, costmap_origin_y, True)
    if (should_be_successful != success):
        print('Fiels "success" is not equal to the awaited value')
        return False
    if success:
        if math.hypot(x1 - x2, y1 - y2) > eps and len(traj) == 0:
            print('The trajectory is empty')
            return False
        # check initial and final points
        if math.hypot(x1 - traj[0][0], y1 - traj[0][1]) > eps:
            print('Initial points do not match')
            return False
        if math.hypot(x2 - traj[-1][0], y2 - traj[-1][1]) > eps:
            print('Final points do not match')
            return False
        #create figure
        fig = pyplot.figure()
        ax = fig.add_subplot()
        #paint trajectory
        ax.plot([v[0] for v in traj], [v[1] for v in traj],'o-')
        #paint start anf goal
        ax.plot(x1, y1, 'gD', x2, y2, 'rD')
        #paint obstacles
        x_o, y_o = obstacle_coordinate.obstacle_coordinate(costmap)
        x_o.remove(100000)
        y_o.remove(100000)
        pyplot.title("Trajectory")
        for i in range (len(x_o)):
            rect = Rectangle((x_o[i], y_o[i]-costmap_resolution), costmap_resolution, costmap_resolution, facecolor='r')
            ax.add_patch(rect)
        #paint formal text
        pyplot.xlabel('x, metres')
        pyplot.ylabel('y, metres')        
        pyplot.grid()
        pyplot.show()
    return True

test_trajectory(-0.0076484298085800605, -0.003595331362215989, 0.040035085910144236, 6.93065071105957, 13.885063171386719, -0.006575553212314842, [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 254, 0, 0], [0, 0, 0, 0, 0, 0, 254, 254, 254, 0], [0, 0, 0, 0, 0, 254, 254, 254, 254, 254], [0, 0, 0, 0, 0, 0, 254, 254, 254, 0], [0, 0, 0, 0, 0, 0, 0, 254, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 254, 0, 0, 0], [0, 0, 0, 0, 0, 254, 254, 254, 0, 254], [0, 0, 0, 0, 254, 254, 254, 254, 254, 254]], 1.0, 0.0, 0.0, True)

exit()

test_trajectory(0, 2, 0, 4, 3, 0, 
                [[0,0,0,0], [0,0,0,0], [0,0,100, 0], [0,0,0,0]],
                1, 0, 0, True)

test_trajectory(0, 2, 0, 4, 3, 0, 
                [[0,0,0,0], [0,0,100,0], [0,0,0, 0], [0,0,0,0], [0,0,0,0] ],
                1, 0, 0, True)

test_trajectory(1, 3.01, 0, 4, 3.01, 0, 
                [[0,0,0,0], [0,0,0,0], [0,0,100,100], [0,0,0,0]],
                1, 0, 0, True)

test_trajectory(1, 1, 0, 3, 3, 0, 
                [[0]*100]*50,
                1, 0, 0, True)

 
test_trajectory(1, 1, 0, 6, 8, 0, 
                [
                    [0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 100, 100, 100, 100, 100],
                    [0, 0, 0, 100, 0, 0, 0],
                    [0, 0, 0, 100, 0, 0, 0],
                    [0, 0, 0, 100, 0, 0, 0],
                    [0, 0, 0, 100, 0, 0, 0],
                    [0, 0, 0, 100, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0]
                ],
                1, 0, 0, True)


