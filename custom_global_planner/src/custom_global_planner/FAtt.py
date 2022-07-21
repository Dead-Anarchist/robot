#!/usr/bin/env python3
import numpy as np


#count Fatt - power of attractive
#dg - vector-string from robot to goal
ka = 0.6 #for attractive power
#rog - radius of zone near goal, where we can use parabolic potention field
rog = 0.27 #metres

def FAtt(x_goal, y_goal, x_r, y_r):
    dg = np.array([x_goal - x_r, y_goal - y_r])

    if np.linalg.norm(dg) <= rog:
        Fatt = ka / rog * dg
    else:
        Fatt = ka * dg / np.linalg.norm(dg)    
    return Fatt
