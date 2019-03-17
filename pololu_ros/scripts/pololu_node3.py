#!/usr/bin/python
from __future__ import division
from __future__ import print_function
from pololu_driver import clip, Pololu # serial controller for Pololu
import multiprocessing
import rospy
import threading
import time
import numpy as np
from time import sleep
from std_msgs.msg import Bool, Float32
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from keyboard_custom.msg import wheelvel 
import sys, os
import select, termios, tty
from mobilenet_detect.msg import cvbox
from mobilenet_detect.msg import cvboxarray

from tracker.msg import TrackedPerson
from tracker.msg import TrackedPersonArray

#commands  = str("a b c d")
#commands = commands.split()


wheel_1_vel = 0
wheel_2_vel = 0

speedBindings={
        'w':(100,100),
        's':(-100,-100),
        'd':(100,-100),
        'a':(-100,100),
        'k':(0,0),
	'g':(0,0),
    }

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def callback(data):
        global key
	global wheel_1_vel
	global wheel_2_vel
	global speedBindings
	key = getKey()
	if key in speedBindings.keys():
		wheel_1_vel += speedBindings[key][0]
		wheel_2_vel += speedBindings[key][1]
		if wheel_1_vel < 0:
			wheel_1_vel = 0
		if wheel_1_vel > 3200:
			whee1_1_vel = 3200
		if wheel_2_vel < 0:
			wheel_2_vel = 0
		if wheel_2_vel > 3200:
			wheel_2_vel = 3200
		if key == 'k':
			wheel_1_vel = 0
			wheel_2_vel = 0
		try:		
			while key == 'g':
				image_width = 640
				image_height = 480
				box = np.zeros(4)
				box[0] = data.objects[0].x_min
				box[1] = data.objects[0].x_max 
				box[2] = data.objects[0].y_min 
				box[3] = data.objects[0].y_max
				print(box)
				bbox_area = (box[1]-box[0])*(box[3]-box[2])
				bbox_area_limit = 200000
				bbox_x_center = (box[0]+box[1])/2
				if bbox_area < bbox_area_limit:
					if (.4*image_width) < bbox_x_center < (.6*image_width):
						wheel_1_vel = 400
						wheel_2_vel = 400
					elif bbox_x_center < (.4*image_width):
						wheel_1_vel = 200
						wheel_2_vel = 600
					elif bbox_x_center > (.6*image_width):
						wheel_1_vel = 600
						wheel_2_vel = 200
			sys.stdout.flush()		
		except KeyboardInterrupt:
    			key ='k'

	print(wheel_1_vel)
	print(wheel_2_vel)
	#os.system("./SmcCmd -d 33FF-6D06-4D4B-3731-4130-1543 --speed " + str(wheel_1_vel))
	#os.system("./SmcCmd -d 33FF-6D06-4D4B-3731-4130-1543 --speed " + str(wheel_2_vel))



if __name__=="__main__":
	settings = termios.tcgetattr(sys.stdin)
	rospy.init_node("pololu_node", log_level=rospy.INFO)
	global key
	key = getKey()
	while key=='q':
		key2 = getKey()
		if key2 in speedBindings.keys():
			wheel_1_vel += speedBindings[key2][0]
			wheel_2_vel += speedBindings[key2][1]
			if wheel_1_vel < 0:
				wheel_1_vel = 0
			if wheel_1_vel > 3200:
				whee1_1_vel = 3200
			if wheel_2_vel < 0:
				wheel_2_vel = 0
			if wheel_2_vel > 3200:
				wheel_2_vel = 3200
			if key2 == 'k':
				wheel_1_vel = 0
				wheel_2_vel = 0
			if key2 == 'f':
				key == 'f'
				print(3)
		print(wheel_1_vel)
		print(wheel_2_vel)
		#os.system("./SmcCmd -d 33FF-6D06-4D4B-3731-4130-1543 --speed " + str(wheel_1_vel))
		#os.system("./SmcCmd -d 33FF-6D06-4D4B-3731-4130-1543 --speed " + str(wheel_2_vel))
	#rospy.Subscriber('tracked_box', TrackedPersonArray, callback)
	rospy.spin()
