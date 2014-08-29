#!/usr/bin/env python
# Irobotique Communication Module : Platform class
#
# Software License Agreement (BSD)
# 
# Gabriel Urbain <gurbain@mit.edu>
# Irobotique ASBL (Belgium) - August 2014

# --------- Physical actuators ---------- 

class Servo:
	"This structure corresponds to the representation of a servomotor"
	OFF = 0
	ON = 1
	
	
	def __init__(self, speed_=0, pos_=0, state_=OFF):
		"Class initialization"
		self.speed = speed_;
		self.pos = pos_;
		self.state = state_;
	
	def set_speed(self, speed_):
		"Set the servomotor speed"
		self.speed = speed_;
	
	def set_pos(self, pos_):
		"Set the servomotor position"
		self.pos = pos_;

	def set_state(self, state_):
		"Set the servomotor state"
		self.state = state_;

class Led:
	"This structure corresponds to the representation of a LED"
	OFF = 0
	ON = 1
	BLINK_L = 2
	BLINK_M = 3
	BLINK_H = 4
	
	def __init__(self, state_=OFF):
		"Class initialization"
		self.state = state_;

	def set_state(self, state_):
		"Set the LED state"
		self.state = state_;

# --------- Physical sensors ------------

class Switch:
	"This structure corresponds to the representation of a simple switch"
	OFF = 0
	ON = 1
	
	def __init__(self, state_=OFF):
		"Class initialization"
		self.state = state_;

	def set_state(self, state_):
		"Set the switch "
		self.state = state_;
	
class Potentiometer:
	"This structure corresponds to the representation of a simple potentiometer"
	
	def __init__(self, value_=0, mini_=0, maxi_=0):
		"Class initialization"
		self.value = value_;
		self.mini = mini_;
		self.maxi = maxi_;
	
	def set_value(self, value_):
		"Set the potentiometer value"
		self.value = value_;
		
	def set_min(self, mini_):
		"Set the potentiometer minimal value"
		self.mini = mini_;
	
	def set_max(self, maxi_):
		"Set the potentiometer maximal value"
		self.maxi = maxi_;

# --------- Robot commands ----------------

class Pltf_ctl:
	"This structure content contains the different user commands from the robotic platform"
	HORI = 0
	VERT = 1
	
	def __init__(self, modif_=False, led1_=Led(), led2_=Led(), led3_=Led(), servo_h_=Servo(), servo_v_=Servo()):
		"Class initialization"
		self.modif = modif_;
		self.led1 = led1_;
		self.led2 = led2_;
		self.led3 = led3_;
		self.servo_h = servo_h_;
		self.servo_v = servo_v_;
		
	def set_led(self, led_, num):
		"Set the led. Variable num represents the LED number. The number 0 corresponds to the first one"
		if num==0:
			self.led1 = led_;
		elif num==1:
			self.led2 = led_;
		elif num==2:
			self.led3 = led_;
		else:
			print "Error";
	
	def get_led(self, num):
		"Get the led. Variable num represents the LED number. The number 0 corresponds to the first one"
		if num==0:
			return self.led1;
		elif num==1:
			return self.led2;
		elif num==2:
			return self.led3;
		else:
			print "Error";
			return -1;
		
	def set_servo(self, servo_, orientation):
		"Set the Servomotor. Variable orientation is set to HORI or VERT depending of which Servomotor to select"
		if orientation==self.HORI:
			self.servo_h = servo_;
		elif orientation==self.VERT:
			self.servo_V = servo_;
		else:
			print "Error";
			
	def get_servo(self, servo_, orientation):
		"Get the Servomotor. Variable orientation is set to HORI or VERT depending of which Servomotor to select"
		if orientation==self.HORI:
			self.servo_h = servo_;
		elif orientation==self.VERT:
			self.servo_V = servo_;
		else:
			print "Error";
		
	def set_modif(self, modif_):
		"Set the modification state to true or false. True when something has been modified and not sent yet"
		self.modif = modif_;

# --------- Robot state -------------------

class Pltf_state:
	"This structure represents the state of the robotic platform and is provided by the server in this one"
	HORI = 0
	VERT = 1
	
	def __init__(self, modif_=False, led1_=Led(), led2_=Led(), led3_=Led(), servo_h_=Servo(), servo_v_=Servo()):
		"Class initialization"
		self.modif = modif_;
		self.led1 = led1_;
		self.led2 = led2_;
		self.led3 = led3_;
		self.servo_h = servo_h_;
		self.servo_v = servo_v_;
		
	def set_led(self, led_, num):
		"Set the led. Variable num represents the LED number. The number 0 corresponds to the first one"
		if num==0:
			self.led1 = led_;
		elif num==1:
			self.led2 = led_;
		elif num==2:
			self.led3 = led_;
		else:
			print "Error";
	
	def get_led(self, num):
		"Get the led. Variable num represents the LED number. The number 0 corresponds to the first one"
		if num==0:
			return self.led1;
		elif num==1:
			return self.led2;
		elif num==2:
			return self.led3;
		else:
			print "Error";
			return -1;
		
	def set_servo(self, servo_, orientation):
		"Set the Servomotor. Variable orientation is set to HORI or VERT depending of which Servomotor to select"
		if orientation==self.HORI:
			self.servo_h = servo_;
		elif orientation==self.VERT:
			self.servo_V = servo_;
		else:
			print "Error";
			
	def get_servo(self, servo_, orientation):
		"Get the Servomotor. Variable orientation is set to HORI or VERT depending of which Servomotor to select"
		if orientation==self.HORI:
			self.servo_h = servo_;
		elif orientation==self.VERT:
			self.servo_V = servo_;
		else:
			print "Error";
		
	def set_modif(self, modif_):
		"Set the modification state to true or false. True when something has been modified and not sent yet"
		self.modif = modif_;