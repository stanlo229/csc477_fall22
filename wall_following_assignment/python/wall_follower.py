#!/usr/bin/env python3
from cmath import inf, nan
import rospy
import tf
from std_msgs.msg import String, Header, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import sqrt, cos, sin, pi, atan2
import numpy
import sys

class PID:
    def __init__(self, Kp, Td, Ti, dt):
        self.Kp = Kp
        self.Td = Td
        self.Ti = Ti
        self.curr_error = 0
        self.prev_error = 0
        self.sum_error = 0
        self.prev_error_deriv = 0
        self.curr_error_deriv = 0
        self.control = 0
        self.dt = dt
        
    def update_control(self, current_error, reset_prev=False):
        # todo: implement this
        term_1 = current_error
        term_2 = self.Td*((current_error - self.prev_error)/self.dt)
        term_3 = (1/self.Ti)*(self.sum_error + current_error)
        if term_3 > 1:
            term_3 = 1.0
        elif term_3 < -1:
            term_3 = -1.0
        print(f"{term_1=}, {term_2=}, {term_3=}")
        # print(f"{term_1=}, {term_2=}")
        self.control = self.Kp*(term_1 + term_2 + term_3)
        if self.control > 0.5:
            self.control = 0.5
        elif self.control < -0.5:
            self.control = -0.5
        # print(f"{self.control}")
        self.prev_error = current_error
        self.sum_error += current_error
        
    def get_control(self):
        return self.control
        
class WallFollowerHusky:
    def __init__(self):
        rospy.init_node('wall_follower_husky', anonymous=True)

        self.forward_speed = rospy.get_param("~forward_speed", 1.5)
        self.desired_distance_from_wall = rospy.get_param("~desired_distance_from_wall", 1)
        self.hz = 50 

        # todo: set up the command publisher to publish at topic '/husky_1/cmd_vel'
        # using geometry_msgs.Twist messages
        self.cmd_pub = rospy.Publisher('/husky_1/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
        # rospy.init_node('talker', anonymous=True)

        self.cte_pub = rospy.Publisher('/husky_1/cte', Float32, queue_size=10)
        # rospy.init_node('cte_talker', anonymous=True)
        # self.cte_sub = rospy.Subscriber()

        # todo: set up the laser scan subscriber
        # this will set up a callback function that gets executed
        # upon each spinOnce() call, as long as a laser scan
        # message has been published in the meantime by another node
        # rospy.init_node('listener', anonymous=True)
        gains = rospy.get_param("/wall_following_assignment")
        K_P, T_I, T_D = gains["K_P"], gains["T_I"], gains["T_D"]
        self.pid = PID(K_P, T_D, T_I, 1)

        self.laser_sub = rospy.Subscriber('/husky_1/scan', LaserScan, self.laser_scan_callback)
    
        
        
    def laser_scan_callback(self, msg):
        # todo: implement this
        # Populate this command based on the distance to the closest
        # object in laser scan. I.e. compute the cross-track error
        # as mentioned in the PID slides.

        # You can populate the command based on either of the following two methods:
        # (1) using only the distance to the closest wall
        # (2) using the distance to the closest wall and the orientation of the wall
        #
        # If you select option 2, you might want to use cascading PID control.
        # 720 is max. Angle ranges from -135 to +135.
        neg_90_scan = msg.ranges[120] - 1

        cte = neg_90_scan - self.desired_distance_from_wall
        if cte == inf:
            cte = 0.3
        self.cte_pub.publish(cte)
        print(f"{cte=}")
        
        self.pid.update_control(current_error=cte)
        msg = Twist()
        msg.linear.x = 1.5
        msg.angular.z = self.pid.get_control()
        self.cmd_pub.publish(msg)
        print(f"{msg.angular.z=}")

        # cmd.angular.z = ?


        pass
   
            
    def run(self):
        rate = rospy.Rate(self.hz)
        while not rospy.is_shutdown():
            rate.sleep()

    
if __name__ == '__main__':
    wfh = WallFollowerHusky()
    wfh.run()


