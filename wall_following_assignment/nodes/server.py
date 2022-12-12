#!/usr/bin/env python3

import rospy

from dynamic_reconfigure.server import Server
from wall_following_assignment.cfg import pidConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: K_P: {K_P},\ 
          T_I: {T_I}, T_D: {T_D}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("wall_following_assignment", anonymous = False)

    srv = Server(pidConfig, callback)
    rospy.spin()