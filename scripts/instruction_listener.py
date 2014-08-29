#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Inspired by talker demo
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Modified by Gabriel Urbain <gurbain@mit.edu>
# for non-profit association irobotique ASBL (Belgium)

## ROS import
import rospy
from std_msgs.msg import String

## Project import
from Pad import *
from Platform import *
import pickle


def callback(msg):
	a = str(msg.data)
	pltf_instn = pickle.loads(a)
	r_str = " Received command:  X: %d et Y: %d" %(pltf_instn.servo_h.speed, pltf_instn.servo_v.speed)
	rospy.loginfo(rospy.get_caller_id()+r_str)
    
def instruction_listener():
	rospy.init_node('instruction_listener', anonymous=True)
	rospy.Subscriber("instruction", String, callback)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()
        
if __name__ == '__main__':
	instruction_listener()
