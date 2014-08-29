#!/usr/bin/env python
# Irobotique Communication Module : Pad class
#
# Software License Agreement (BSD)
# 
# Gabriel Urbain <gurbain@mit.edu>
# Irobotique ASBL (Belgium) - August 2014

import pygame
import sys
import time
import threading
import termios, os
from Platform import *

class Joystick(threading.Thread):
	"This class provides the tools for the Joystick utilisation in the Client interface"
	PADUNRECOGN = 0
	PADUSBGEN = 1
	
	def __init__(self, pltf_):
		"Class initialization"
		threading.Thread.__init__(self)
		self.isTerminated = False
		self.pltf = pltf_
		pygame.init()
		pygame.joystick.init()
		if self.isJoystick()==True:
			self.defType()
		
	def isJoystick(self):
		"This function help to choose wich Joystick to use when plugged"
		nb_joysticks = pygame.joystick.get_count()
		if nb_joysticks >1:
			print "Choose which Joystick you want to use (between 1 and %d):" % nb_joysticks,
			n = input();
			self.j = pygame.joystick.Joystick(n-1)
			self.j.init()
			return True;
		elif nb_joysticks == 1:
			self.j = pygame.joystick.Joystick(0)
			self.j.init()
			return True;
		else:
			print "No Joystick plugged! Verify the connection and the recognition by the OS..."
			return False;
			
	def defType(self):
		if self.j.get_name().find("Microntek              USB Joystick") != -1:
			self.jType = self.PADUSBGEN;
		else:
			self.jType = self.PADUNRECOGN;
	
	def dispFeatures(self):
		"This function display the Joystick features on screen"
		print "Axes :", self.j.get_numaxes()
		print "Boutons :", self.j.get_numbuttons()
		print "Trackballs :", self.j.get_numballs()
		
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
			for event in pygame.event.get():
				if event.type == pygame.JOYBUTTONDOWN:
					if event.button == 0:
						print "Triangle"
					elif event.button == 1:
						print "Rond"
					elif event.button == 2:
						print "Croix"
					elif event.button == 3:
						print "Carre"
					elif event.button == 4:
						print "L1"
					elif event.button == 5:
						print "R1"
					elif event.button == 6:
						print "L2"
					elif event.button == 7:
						print "R2"
					elif event.button == 8:
						print "Select"
					elif event.button == 9:
						print "Start"
					elif event.button == 10:
						print "Pad1"
					elif event.button == 11:
						print "Pad2"
					else:
						print "Unknow"
				if event.type == pygame.JOYAXISMOTION:
					X1 = self.j.get_axis(0)
					Y1 = - self.j.get_axis(1)
					X2 = self.j.get_axis(2)
					Y2 = - self.j.get_axis(3)
					self.pltf.servo_h.set_speed(X1*10)
					self.pltf.servo_v.set_speed(Y1*10)
					self.pltf.modif = True
				if event.type == pygame.JOYHATMOTION:
					(X3, Y3) = self.j.get_hat(0)
			
			# Display values
			#print "Axe 1: X = {:>3f} et Y = {:>3f}".format(X1,Y1),
			#print "  ||  Axe 2: X = {:>3f} et Y = {:>3f}".format(X2,Y2),
			#print "  ||  Axe 3: X = {:d} et Y = {:d}".format(X3,Y3)
			
	def stop(self):
		self.isTerminated = True;
		
class Keyboard(threading.Thread):
	"This class provides the tools for the Joystick utilisation in the Client interface"
	
	def __init__(self, pltf_):
		"Class initialization"
		threading.Thread.__init__(self)
		self.isTerminated = False
		self.pltf = pltf_
		
	def getkey(self):
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
		
		i = 0
		while not self.isTerminated:
			# Collect the values from the gamepad
			time.sleep(0.1)
			self.pltf.modif = False
			c = self.getkey();
			if c=='e':
				self.pltf.modif = True
				self.pltf.servo_v.set_speed(10)
				self.pltf.servo_h.set_speed(0)
				print "Avancer"
				i = 0
			elif c=='d':
				self.pltf.modif = True
				self.pltf.servo_v.set_speed(-10)
				self.pltf.servo_h.set_speed(0)
				print "Reculer"
				i = 0
			elif c=='s':
				self.pltf.modif = True
				self.pltf.servo_h.set_speed(-10)
				self.pltf.servo_v.set_speed(0)
				print "Gauche"
				i = 0
			elif c=='f':
				self.pltf.modif = True
				self.pltf.servo_h.set_speed(10)
				self.pltf.servo_v.set_speed(0)
				print "Droite"
				i = 0
			else:
				self.pltf.modif = True
				self.pltf.servo_h.set_speed(0)
				self.pltf.servo_v.set_speed(0)
				
	def stop(self):
		self.isTerminated = True;