#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist 
from gazebo_msgs.msg import ModelStates
import math
import random
import numpy as np

from nav_msgs.msg import OccupancyGrid 

def new_coord():
    'count koordintes of goal'
    return (random.randint(-5,5), random.randint(-5,5))

class Controller(object):
    def __init__(self):
        # state variables
        self.current_goal = new_coord()
        rospy.loginfo('New goal: {},{}'.format(self.current_goal[0], self.current_goal[1]))
        self.start_time = rospy.Time.now()
        self.last_msg = None
        
        # ROS infrastructure
        self.velocity_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size = 1)
        self.pose_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.cb_pose)
        self.costmap_sub = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.cb_costmap)
        rospy.loginfo('Controller: initialization completed')
    
    def cb_costmap(self, msg):
        self.scale = msg.info.resolution #masshtab
        self.width = msg.info.width #cells
        self.height = msg.info.height #cells
        
        cost_list = msg.data #list of costs
        
        #parsing from list to matrix 2x2 in cells
        a = 0
        b = self.width-1
        cost_mat_cells = np.array()
        for i in range (0, self.height):
            cost_mat_cells.append(msg.data[a:b])
            a+=self.width
            b+=self.width
        
        cost_mat_m = cost_mat_cells * self.scale #matrix in metres
        
        x_o = np.array()
        y_o = np.array()
        for i in range (0, self.height*self.scale):
            for j in range (0, self.width*self.scale):
                if cost_mat_m[i,j] > 50:
                    x_o.append(j)
                    y_o.append(i)
        
    def cb_pose(self, msg):
        self.last_msg = msg
        
    def min_skal(do_list):
        skals = np.array()
        for i in range (    0, do_list.length):
            skals.append(np.linalg.norn(do_list[i]))
    
        index = skals.index(min(scals))
        return index
    
    def robot_bug_nav(self): 
        if self.last_msg is None:
            return
        
        robot_name = 'test_robot'
        
        if robot_name not in self.last_msg.name:
            return
        robot_index = self.last_msg.name.index(robot_name)
        
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

        #constants for DU Tr'' + r' = ku, u - control impact. u = Fatt, if we have free moving mode
        #u = Ftan + Frep, if we have bypass mode
        T = 0.2 
        k = 1
        
        pos_rob = self.last_msg.pose[robot_index].position
        
        #x_r y_r - coordinates of robot's position
        x_r = pos_rob.x
        y_r = pos_rob.y
        #x_goal y_goal - coordinates of end (goal) robot's position 
        x_goal = self.current_goal[0]
        y_goal = self.current_goal[1]

        #initial conditions for robot
        x_start = pos_rob.x
        y_start = pos_rob.y

        ori = self.last_msg.pose[robot_index].orientation
        
        #found obstacles
        
        do_list = np.array()
        
        for i in range (0, x_o.length):
            do_list.append([x_o[i] - x_r, y_o[i] - y_r])
        
        #new_msg - output of this function - new massage with list of points for way from start to goal for ROS
        output_list = []
        output_list.append([x_start, y_start]) #app start pozition
        
        #count Fatt - power of attractive
        #dg - vector-string from robot to goal
        dg = np.array([x_goal - x_r, y_goal - y_r])

        if np.linalg.norm(dg) <= rog:
            Fatt = ka / rog * dg
        else:
            Fatt = ka * dg / np.linalg.norm(dg)
            
        #count Frep - power of repelling from obstacles
        #do - vector-string from robot to obstacle, x_o y_o - obstacle coordinates 
        do = do_list[min_skal(do_list)]

        if np.linalg.norm(do) <= ror:
            Frep = -kr * (1/np.linalg.norm(do) - 1/ror) * do/np.linalg.norm(do)**3
        else:
            Frep = 0

        #count of Ftan - tangencial power for bypass obstacle

        Ftan = np.array([-do[1] * np.linalg.norm(Fatt) / np.linalg.norm(do), do[0] * np.linalg.norm(Fatt) / np.linalg.norm(do) ])

        #DU in normal form Koshi, r is r' in our task
        
        def func(r, u, T, k):
            dydt = -1/T * r + k/T * u
            return dydt
        
        #let t (time argument) change in range 0 to 10 sec, 101 points
        t = np.linspace(0, 10, 101)
        
        #we have 3 variables of system condition. 1) robot can move free - free mode
        #2) robot bypass obstacle - bypass mode
        #3) robot comes goal
        
        while np.linalg.norm(dg) > 0.1: #let comfort distance to goal is 10 cm
            if np.dot(dg, do) < 0:
                if np.linalg.norm(dg) < np.linalg.norm([r_pl[0] - x_goal, r_pl[1] - y_goal]):
                    if (np.linalg.norm(do) >= ro0_pl or np.linalg.norm(do) >= ror):
                        u = Fatt
                    else:
                        r_pl = pos_rob
            else:
                u = Ftan + Frep
            
            r0 = np.array([x_start, y_start])
            r_sol = odeint(func, r0, t)
            
            output_list.append(r_sol)
        
        rospy.logerr('we work'.format())
        return output_list
    
    
if __name__ == '__main__':
    try:
        rospy.init_node('controller', anonymous=True)
        rospy.loginfo('Controller: start')
        c = Controller()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
