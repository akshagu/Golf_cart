#!/usr/bin/env python

#Provides Keyboard Commands. Built specifically for publishing to rosserial node

from __future__ import print_function
import roslib
roslib.load_manifest('keyboard_custom')
import rospy
import multiprocessing
import os
import sys
import time
from time import sleep
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from keyboard_custom.msg import wheelvel
import std_msgs.msg
import sys, select, termios, tty


speedBindings={
        'w':(100,100),
        's':(-100,-100),
        'd':(100,-100),
        'a':(-100,100),
        'k':(0,0),
    }

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    pub = rospy.Publisher('keyboard_velocity', wheelvel)
    rospy.init_node('keyboard_output')
    wheel_1_vel = 0
    wheel_2_vel = 0
    while(1):        
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
        elif (key == '\x03'):
                    break
        #else:
	
        



        msg = wheelvel()
        msg.one = wheel_1_vel
	msg.two = wheel_2_vel
	pub.publish(msg)
