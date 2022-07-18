#!/usr/bin/env python3
 
import custom_global_planner_function as cgpf
import math 
def test_trajectory(x1, y1, ori1,
                    x2, y2, ori2,
                    costmap, costmap_resolution,
                    costmap_origin_x, costmap_origin_y,
                    should_be_successful, eps = 0.1):
    # trying to trace a trajectory
    success, traj = cgpf.make_trajectory(x1, y1, ori1,
                    x2, y2, ori2,
                    costmap, costmap_resolution,
                    costmap_origin_x, costmap_origin_y)
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
    return True



test_trajectory(1, 1, 0, 4, 4, 0, 
                [[0,0,0,0], [0,0,0,0], [0,0,100,0], [0,0,0,0]],
                1, 0, 0, True)


exit()
test_trajectory(1, 1, 0, 3, 3, 0, 
                [[0]*100]*50,
                1, 0, 0, True)

 
test_trajectory(1, 1, 0, 3, 3, 0, 
                [[100]*5,[100]+[0]*3+[100],[100]+[0]*3+[100],[100]+[0]*3+[100],[100]*5],
                1, 0, 0, True)


