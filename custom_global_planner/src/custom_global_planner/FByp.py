#!/usr/bin/env python3
import numpy as np
import FAtt
#count Fbyp = Frep + Ftan
#Frep - power of repelling from obstacles
#Ftan - tangencial power for bypass obstacle
#do - vector-string from robot to obstacle, x_o y_o - obstacle coordinates 
def FByp(do, x_r, y_r, x_goal, y_goal):
    kr = 1e-03 #for repelling power

    #ror - radius of zone near obstacle, where potention value growing up so much
    ror = 3 #metres
    if np.linalg.norm(do) <= ror:
        Frep = -kr * (1/np.linalg.norm(do) - 1/ror) * do/np.linalg.norm(do)**3
    else:
        Frep = [0,0]
        
    Fatt = FAtt.FAtt(x_goal, y_goal, x_r, y_r)
    Ftan = np.array([-do[1] * np.linalg.norm(Fatt) / np.linalg.norm(do), do[0] * np.linalg.norm(Fatt) / np.linalg.norm(do)])
    return Frep + Ftan
