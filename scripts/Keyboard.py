#!/usr/bin/env python
# Irobotique Communication Module : Keyboard input class test
#
# Software License Agreement (BSD)
# 
# Gabriel Urbain <gurbain@mit.edu>
# Irobotique ASBL (Belgium) - August 2014

import sys
import time
import threading
from Platform import *

class Keyboard(threading.Thread):
	"This class provides the tools for the Joystick utilisation in the Client interface"
	
	def __init__(self, pltf_):
		"Class initialization"
		threading.Thread.__init__(self)
		self.isTerminated = False
		self.pltf = pltf_
		
	def getkey():
		"Get terminal key in non-canonical mode"
		term = open("/dev/tty", "r")
		fd = term.fileno()
		old = termios.tcgetattr(fd)
		new = termios.tcgetattr(fd)
		new[3] &= ~termios.ICANON & ~termios.ECHO
		termios.tcsetattr(fd, termios.TCSANOW, new)
		c = None
		try:
			c = os.read(fd, 1)
		finally:
			termios.tcsetattr(fd, termios.TCSAFLUSH, old)
			term.close()
		return c

	def run(self):
		"This function check if there is an event on the pad. If yes, it fill the command structure, which can be send then"
		X1 = 0.0
		Y1 = 0.0
		X2 = 0.0
		Y2 = 0.0
		X3 = 0
		Y3 = 0
		while not self.isTerminated:
			
			# Sleep is required for an optimized processor consumption
			time.sleep(0.1)
			
			# Collect the values from the gamepad
			c = self.getkey();
			print "C est ma cle %s"%c
			
	def stop(self):
		self.isTerminated = True;