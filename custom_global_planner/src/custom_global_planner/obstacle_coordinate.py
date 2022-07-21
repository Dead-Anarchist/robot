#!/usr/bin/env python3
#find coordinates of obstacles in metres
import numpy as np

def obstacle_coordinate(costmap):
    cost_mat_cells = np.array(costmap)
    height, width = cost_mat_cells.shape
    
    x_o = [100000]
    y_o = [100000]
    
    for i in range (width):
        for j in range (height):
            if cost_mat_cells[j,i] > 50:
                x_o.append(i)
                y_o.append(height-j-1)
                
    return [x_o, y_o]
