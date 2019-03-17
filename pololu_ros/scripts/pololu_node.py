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
        'w':(400,400),
        's':(-100,-100),
        'd':(400,-400),
        'a':(-400,400),
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
        print(data)
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
		if key == 'g':
			image_width = 1280
			image_height = 720
			box = np.zeros(4)
			box[0] = data.objects[0].x_min
			box[1] = data.objects[0].x_max 
			box[2] = data.objects[0].y_min 
			box[3] = data.objects[0].y_max
			#print(box)
			bbox_area = np.abs((box[1]-box[0])*(box[3]-box[2]))
			bbox_area_limit = 250000
			bbox_x_center = (box[0]+box[1])/2
			#print(bbox_area, 'person area')
			center_speed = 1200
			if bbox_area < bbox_area_limit:
				wheel_1_vel = int(center_speed * (1-(image_width/2-bbox_x_center)/(image_width/2)))
				wheel_2_vel = int(center_speed * (1-(-image_width/2+bbox_x_center)/(image_width/2)))
				'''if (.2*image_width) < bbox_x_center < (.8*image_width):
					print('person center')
					wheel_1_vel = 1200
					wheel_2_vel = 1200
				elif bbox_x_center < (.2*image_width):
					print('person left')
					wheel_1_vel = 400
					wheel_2_vel = 2000
				elif bbox_x_center > (.8*image_width):
					print('person right')
					wheel_1_vel = 2000
					wheel_2_vel = 400'''
			elif bbox_area > bbox_area_limit:
				wheel_1_vel = 0
				wheel_2_vel = 0
			
	#print(wheel_1_vel)
	#print(wheel_2_vel)
	os.system("mono /home/ubuntu/catkin_ws/src/pololu_ros/smc_linux/SmcCmd -d 33FF-6D06-4D4B-3731-4130-1543 --speed " + str(-wheel_2_vel))
	os.system("mono /home/ubuntu/catkin_ws/src/pololu_ros/smc_linux/SmcCmd -d 33FF-7006-4D4B-3731-1531-1543 --speed " + str(wheel_1_vel))



if __name__=="__main__":
	settings = termios.tcgetattr(sys.stdin)
	rospy.init_node("pololu_node", log_level=rospy.INFO)
        os.system("mono /home/ubuntu/catkin_ws/src/pololu_ros/smc_linux/SmcCmd -d 33FF-6D06-4D4B-3731-4130-1543 --resume")
	os.system("mono /home/ubuntu/catkin_ws/src/pololu_ros/smc_linux/SmcCmd -d 33FF-7006-4D4B-3731-1531-1543 --resume")

	rospy.Subscriber('tracked_box', TrackedPersonArray, callback,queue_size=1)
	#image_sub = Subscriber("/v4l/camera/image_raw",Image,queue_size=1)
	#rospy.Subscriber('cv_test_box',cvboxarray,callback,queue_size=1)
	rospy.spin()
