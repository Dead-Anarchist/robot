#!/usr/bin/env python3


import numpy as np
from scipy.integrate import odeint
from scipy import integrate

#DU in normal form Koshi, r is r' in our task        
def func(r, time, u, T, k):
    dydt = -1/T * r + k/T * u
    return dydt

def min_skal(do_list):
        skals = []
        for i in range (0, len(do_list)):
            skals.append(np.linalg.norm(do_list[i]))
        index = skals.index(min(skals))
        return index
    
def make_trajectory(x1, y1, ori1,
                    x2, y2, ori2,
                    costmap, costmap_resolution,
                    costmap_origin_x, costmap_origin_y):
    print(x1, y1, ori1,
                    x2, y2, ori2,
                    costmap, costmap_resolution,
                    costmap_origin_x, costmap_origin_y) 
    
    
    
    cost_mat_cells = np.array(costmap)
    width, height = cost_mat_cells.shape
    #find coordinates of obstacles in metres
    print('0000', cost_mat_cells.shape)
    x_o = [100000]
    y_o = [100000]
    
    for i in range (width):
        for j in range (height):
            if cost_mat_cells[i,j] > 50:
                x_o.append(i)
                y_o.append(j)
                
    
    print('01')
    print('02')    
    #constants for count potention power
    ka = 0.6 #for attractive power
    kr = 0.12 #for repelling power

    #rog - radius of zone near goal, where we can use parabolic potention field
    rog = 0.27 #metres
    #ror - radius of zone near obstacle, where potention value growing up so much
    ror = 0.18 #metres
    #if there are no obstacles in radius ro0_pl, we have free moving, ro0_pl > ror - width of zone of obstacle influence
    ro0_pl = 0.5 #metres
    #like ro0_pl, but we use ro0_min in bypass mode
    ro0_min = 2 #metres
    rog_stop = 0.1 #radius of zone, where we think, that goal is roached
    ror_max = 2.2 #if nearest obstacle in this radius, we ignore this
    
    #constants for DU Tr'' + r' = ku, u - control impact. u = Fatt, if we have free moving mode
    #u = Ftan + Frep, if we have bypass mode
    T = 0.2 
    k = 1
        
    #x_r y_r - coordinates of robot's position
    x_r = x1
    y_r = y1
    #x_goal y_goal - coordinates of end (goal) robot's position 
    x_goal = x2
    y_goal = y2

    #initial conditions for robot
    x_start = x1 
    y_start = y1

    #new_msg - output of this function - new massage with list of points for way from start to goal for ROS
    output_list = []
    output_list.append((x_start, y_start, ori1)) #app start pozition
            
        
    #let t (time argument) change in range 0 to 10 sec, 101 points
    t = np.linspace(0, 1, 11)
        
    #we have 3 variables of system condition. 1) robot can move free - free mode
    #2) robot bypass obstacle - bypass mode
    #3) robot comes goal
    print('1')
    dg = np.array([x_goal - x_r, y_goal - y_r])
    do_list = []
    for i in range (0, len(x_o)):
        do_list.append([x_o[i]*costmap_resolution - x_r, y_o[i]*costmap_resolution - y_r])
    
    ind = min_skal(do_list)
    do = do_list[ind]
    ################################################################ berem vector do, normiruem, otkadivaem ot prepyztstviya s - znakom
    
    do_norm = do/np.linalg.norm(do)
    r_pl = -ro0_pl * do_norm + do
    

    print('11')
    iter_ = 0
    velocity_x = []
    velocity_x.append(0)
    velocity_y = []
    velocity_y.append(0)
    while np.linalg.norm(dg) > rog_stop: #let comfort distance to goal is 10 cm
        iter_ += 1
        if iter_ > 3:
            break
        print('12')
        print(dg, 'dg')
        print(do, 'do')
        
        #count Fatt - power of attractive
        #dg - vector-string from robot to goal
        dg = np.array([x_goal - x_r, y_goal - y_r])

        if np.linalg.norm(dg) <= rog:
            Fatt = ka / rog * dg
        else:
            Fatt = ka * dg / np.linalg.norm(dg)
            
        if np.dot(dg, do) < 0:
            print('13')
            if (np.linalg.norm(do) >= ro0_pl or np.linalg.norm(do) >= ror):
                u = Fatt
            else:
                u = Fatt #TODO count frep and past in this
        else:
            #count Frep - power of repelling from obstacles
            #do - vector-string from robot to obstacle, x_o y_o - obstacle coordinates 
            print('14')
            do = do_list[min_skal(do_list)]

            if np.linalg.norm(do) >= ror_max:
                u = Fatt
            else:
                if np.linalg.norm(do) <= ror:
                    Frep = -kr * (1/np.linalg.norm(do) - 1/ror) * do/np.linalg.norm(do)**3
                else:
                    Frep = 0

                #count of Ftan costmap_origin_x, costmap_origin_y):- tangencial power for bypass obstacle

                Ftan = np.array([-do[1] * np.linalg.norm(Fatt) / np.linalg.norm(do), do[0] * np.linalg.norm(Fatt) / np.linalg.norm(do)])
                
                u = Ftan + Frep
        print('2')
        
        r0 = np.array([velocity_x[-1], velocity_y[-1]])
        r_sol = odeint(lambda x, time_: func(x, time_, u, T, k), r0, t)
        print(r_sol, "r_sol")
        
        velocity_x = r_sol[:,0]
        x_r += integrate.trapz(velocity_x, t)
        velocity_y = r_sol[:,1]
        y_r += integrate.trapz(velocity_y, t)
        
        print(x_r, "x_r after while", y_r, "y_r after while")
        dg = np.array([x_goal - x_r, y_goal - y_r])
        
        if np.linalg.norm(dg) >= rog_stop:
            output_list.append((x_r, y_r, 0))
        else:    
            x_r -= 0.5*integrate.trapz(velocity_x, t)
            y_r -= 0.5*integrate.trapz(velocity_y, t)
            output_list.append((x_r, y_r, 0))
        print(dg, "dg after while")
        #found obstacles        
        
        for i in range (0, len(x_o)):
            do_list.append([x_o[i]*costmap_resolution - x_r, y_o[i]*costmap_resolution - y_r])
        
    print('we work')
    output_list.append((x_goal, y_goal, ori2))
    print(output_list, 'output')
    return (True, output_list)    

