#!/usr/bin/env python
# Irobotique Communication Module : Instruction talker
#
# Software License Agreement (BSD)
# 
# Gabriel Urbain <gurbain@mit.edu>
# Irobotique ASBL (Belgium) - August 2014


## ROS import
import rospy
from std_msgs.msg import String

## Project import
from Pad import *
from Platform import *
import threading, time
import signal
import sys
import pickle


def signal_handler(signal, frame):
        print('\nUser has exited Irobotique Comm Talker with CTRL-C!')
        #joystick.stop();
        keyboard.stop()
        sys.exit(0)

def instruction_talker(pltf_):
	"This method send the commands through a ROS topic"
	
	# Initialization
	pltf = pltf_
	pub = rospy.Publisher('instruction', String, queue_size=10)
	rospy.init_node('instruction_talker', anonymous=True)
	r = rospy.Rate(5) # 5 Hz
		
	# Sending loop
	while not rospy.is_shutdown():
		if pltf_ctl.modif==True:
			#s_str = "Send command:  X: ", pltf.servo_h.speed, " et Y: ", pltf.servo_v.speed
			#rospy.loginfo(s_str)
			msg = pickle.dumps(pltf);
			pub.publish(msg)
			r.sleep()


if __name__ == '__main__':
	"This main function launch every threads and manage exceptions"
	
	# Initialization
	pltf_state = Pltf_state()
	pltf_ctl = Pltf_ctl()
	signal.signal(signal.SIGINT, signal_handler)

	# Launch Joystick Thread
	try:
		#joystick = Joystick(pltf_ctl)
		#joystick.start()
		keyboard = Keyboard(pltf_ctl)
		keyboard.start();
	except (KeyboardInterrupt, SystemExit): pass
	
	# Launch main loop
	try:
		instruction_talker(pltf_ctl)
	except rospy.ROSInterruptException: pass
	