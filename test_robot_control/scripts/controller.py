#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist 
from gazebo_msgs.msg import ModelStates
import math
import random

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
        self.timer = rospy.Timer(rospy.Duration(0.05), self.cb_timer)
        rospy.loginfo('Controller: initialization completed')
    
    def cb_pose(self, msg):
        self.last_msg = msg
        
    def cb_timer(self, e):
        if self.last_msg is None:
            return
        
        xg = self.current_goal[0]
        yg = self.current_goal[1]
        
        robot_name = 'test_robot'
        
        if robot_name not in self.last_msg.name:
            return
        robot_index = self.last_msg.name.index(robot_name)
        
        ori = self.last_msg.pose[robot_index].orientation
        angle_turn = 2*math.atan2(ori.z, ori.w)
        
        
        pos_rob = self.last_msg.pose[robot_index].position        
        x_dist = xg - pos_rob.x
        y_dist = yg - pos_rob.y
        angle_g = math.atan2(y_dist, x_dist)
        
        angl_move = angle_diff(angle_g, angle_turn)
        
        # if angle is too big rotate to the goal
        out_msg = Twist()
        if abs(angl_move) > 0.02:
            out_msg.linear.x = 0
            out_msg.angular.z = -1 if angl_move < 0 else 1
        else:
            out_msg.angular.z = 0
            
            length = math.hypot(x_dist, y_dist)
            
            if length > 0.2:
                out_msg.linear.x = 1
            else:
                out_msg.linear.x = 0
                # create new goal
                self.current_goal = new_coord()
                duration = (rospy.Time.now() - self.start_time).to_sec()
                self.start_time = rospy.Time.now()
                rospy.loginfo('Time: {}; new goal: {},{}'.format(duration,
                             self.current_goal[0], self.current_goal[1]))
        self.velocity_pub.publish(out_msg)
        
        #rospy.logerr('angl_move: {:.3f} angle_g: {:.3f} angle_turn {:.3f} cmd: {}'.format(angl_move, angle_g, angle_turn, out_msg.angular.z))
        
        

def angle_diff(a1, a2):
    """ difference between 2 angles """
    diff = a1 - a2
    return (diff + math.pi) % (2*math.pi) - math.pi    

if __name__ == '__main__':
    try:
        rospy.init_node('controller', anonymous=True)
        rospy.loginfo('Controller: start')
        c = Controller()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    
