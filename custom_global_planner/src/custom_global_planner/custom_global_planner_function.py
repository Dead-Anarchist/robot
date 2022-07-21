#!/usr/bin/env python3


import numpy as np
from scipy.integrate import odeint
from scipy import integrate
import FAtt, FByp, bypass, obstacle_coordinate, min_skal, du, zone_point, where_square, new_point

def make_trajectory(x1, y1, ori1,
                    x2, y2, ori2,
                    costmap, costmap_resolution,
                    costmap_origin_x, costmap_origin_y, debug_mode = False):
    try:
        return _make_trajectory(x1, y1, ori1,
                    x2, y2, ori2,
                    costmap, costmap_resolution,
                    costmap_origin_x, costmap_origin_y)
    except Exception as e:
        if debug_mode:
            raise e
        else:
            print(e)
            return (False, [])
    except:
        if debug_mode:
            raise e
        else:
            print('unknown error')
            return (False, [])
    
def _make_trajectory(x1, y1, ori1,
                    x2, y2, ori2,
                    costmap, costmap_resolution,
                    costmap_origin_x, costmap_origin_y):
    
    print(x1, y1, ori1,
                    x2, y2, ori2,
                    costmap, costmap_resolution,
                    costmap_origin_x, costmap_origin_y) 
    
    x_o, y_o = obstacle_coordinate.obstacle_coordinate(costmap)
   
    kr = 1e03 #for repelling power

    #ror - radius of zone near obstacle, where potention value growing up so much
    ror = 3 #metres
   
    #if there are no obstacles in radius ro0_pl, we have free moving, ro0_pl > ror - width of zone of obstacle influence
    ro0_pl = 0.5 #metres
    #like ro0_pl, but we use ro0_min in bypass mode
    ro0_min = 2 #metres
    rog_stop = 0.1 #radius of zone, where we think, that goal is roached
    ror_max = 3 #if nearest obstacle in this radius, we ignore this
    
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

    dg = np.array([x_goal - x_r, y_goal - y_r])
    do_list = []
    for i in range (0, len(x_o)):
        do_list.append([x_o[i]*costmap_resolution - x_r, y_o[i]*costmap_resolution - y_r])
    print()
#    print(do_list, "do_list")
    ind, do = min_skal.min_skal(do_list, costmap_resolution)
    #berem vector do, normiruem, otkadivaem ot prepyztstviya s - znakom
    do_norm = do/np.linalg.norm(do)
    r_pl = -ro0_pl * do_norm + do
    
    
    iter_ = 0
    velocity_x = []
    velocity_x.append(0)
    velocity_y = []
    velocity_y.append(0)
    
    while np.linalg.norm(dg) > rog_stop: #let comfort distance to goal is 10 cm
        '''
        iter_ += 1
        if iter_ > 50:
            break
        '''

        print(dg, 'dg')
        print(do, 'do')
        
        reserve_dg = dg #save previos dg for backstep in case of coming in zone 5
        reserve_do = do #save previos do for backstep in case of coming in zone 5
        reserve_rob = [x_r, y_r] #save previos koordinates of robot for backstep in case of coming in zone 5
        
        Fatt = FAtt.FAtt(x_goal, y_goal, x_r, y_r)
     
         
        if np.dot(dg, do) <= 0:
            u = Fatt
            print(u, "u Fatt")
        else:
            if np.linalg.norm(do) >= ro0_pl: #obstacle on the big distance
                u = Fatt
                print(u, "u Fatt")
            else: #obstacle near us
                if np.linalg.norm(dg) <= np.linalg.norm([r_pl[0] - x_goal, r_pl[1] - y_goal]): #find optimal point of turn
                    r_pl = [x_r, y_r]
            
                Fbyp = FByp.FByp(do, x_r, y_r, x_goal, y_goal)
                
                u = [Fatt[0] + Fbyp[0], Fatt[1] + Fbyp[1]]

                print(u, 'u')

        x_r, y_r = new_point.new_point(velocity_x, velocity_y, x_r, y_r, t, u, T, k)
        print(x_r, "x_r after while", y_r, "y_r after while")
        output_list.append((x_r, y_r, 0))
        dg = np.array([x_goal - x_r, y_goal - y_r])
        
        #TODO check_goal()
        '''
        if np.linalg.norm(dg) >= rog_stop:
            output_list.append((x_r, y_r, 0))
        else:
            print('?'*100, x_r, velocity_x)
            x_r -= 0.5*integrate.trapz(velocity_x, t)
            y_r -= 0.5*integrate.trapz(velocity_y, t)
            output_list.append((x_r, y_r, 0))
        print(dg, "dg after while")
        '''
        
        #found obstacles
        do_list = []
        for i in range (0, len(x_o)):
            do_list.append([x_o[i]*costmap_resolution - x_r, y_o[i]*costmap_resolution - y_r])
        ind, do = min_skal.min_skal(do_list, costmap_resolution)
        print(do, "do after while")
        
        #TODO bybass()
        if zone_point.zone_point(x_o[ind]*costmap_resolution, y_o[ind]*costmap_resolution, costmap_resolution, x_r, y_r) == 5:
            do = reserve_do
            dg = reserve_dg
            if np.linalg.norm(do) <= ror:
                Frep = -kr * (1/np.linalg.norm(do) - 1/ror) * do/np.linalg.norm(do)**3
            else:
                Frep = [0, 0]
            Ftan = np.array([-do[1] * np.linalg.norm(Fatt) / np.linalg.norm(do), do[0] * np.linalg.norm(Fatt) / np.linalg.norm(do)])
            u = Frep + Ftan
            x_r = reserve_rob[0]
            y_r = reserve_rob[1]
            
            x_y, y_r = new_point.new_point(velocity_x, velocity_y, x_r, y_r, t, u, T, k)
            print(x_r, "x_r OBHOD", y_r, "y_r OBHOD")
            
            output_list[-1] = (x_r, y_r, 0)
        print()
        print()
        
    output_list.append((x_goal, y_goal, ori2))
    print()
    print(output_list, 'output')
    print('we work')
    return (True, output_list)    

