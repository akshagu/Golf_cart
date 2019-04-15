#!/usr/bin/python

import sys
import time
import logging
import argparse
import rospy
import std_msgs.msg
import random
import sys, os
import select, termios, tty
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import Header
import numpy as np
import cv2

from mobilenet_detect.msg import cvbox
from mobilenet_detect.msg import cvboxarray

from tracker.msg import TrackedPerson
from tracker.msg import TrackedPersonArray

from message_filters import ApproximateTimeSynchronizer, TimeSynchronizer, Subscriber

class select_person:
    
    def __init__(self):
        self.settings = termios.tcgetattr(sys.stdin)
        rospy.init_node('select_person', anonymous=False)
        self.bridge = CvBridge()
	image_sub = Subscriber("/v4l/camera/image_raw",Image,queue_size=1)
	detected_array_sub = Subscriber('cv_test_box',cvboxarray,queue_size=1)
	ats = ApproximateTimeSynchronizer([image_sub, detected_array_sub], 5,.4)
	#tss = TimeSynchronizer(Subscriber("/usb_cam/image_raw",Image), Subscriber('/dnn_objects',DetectedObjectArray))
	ats.registerCallback(self.detector_callback)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        self.key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        print('getKey', self.key)

    def get_person(self,image,detected_array):
        person_count = 0

        for i in range(len(detected_array.objects)):
            if detected_array.objects[i].class_name == 'person':
                person_count += 1

        person_boxes = np.zeros((person_count,4))
	person_images = []

        if person_count > 0:
            person_count = 0
            for i in range(len(detected_array.objects)):
                if detected_array.objects[i].class_name == 'person':
                    coordinates = np.zeros(4)
                    coordinates[0] = detected_array.objects[i].x_min
                    if coordinates[0] < 0:
                        coordinates[0] = 0
                    coordinates[1] = detected_array.objects[i].x_max
                    if coordinates[1] > self.image_width:
                        coordinates[1] = self.image_width
                    coordinates[2] = detected_array.objects[i].y_min
                    if coordinates[2] < 0:
                        coordinates[2] = 0
                    coordinates[3] = detected_array.objects[i].y_max
                    if coordinates[3] > self.image_height:
                        coordinates[3] = self.image_height

                    this_box_image = image[(int(coordinates[2])):(int(coordinates[3])), int(coordinates[0]):int(coordinates[1])]

                    #Z = this_box_image.reshape((-1,3))
                    #Z = np.float32(this_box_image)  
                    person_images.append(this_box_image)

                    person_boxes[person_count] = coordinates

                    person_count += 1


            for i in range(person_count):
                cv2.imshow('Person '+str(i+1),person_images[i])
                cv2.waitKey(0)
                cv2.destroyAllWindows()

            person_number = -1
            while (person_number < 1) or (person_number > (person_count+1)):
                person_number = int(raw_input("Please enter which person you wish to follow: "))


            if person_number > 0:
                self.tracking = True
                self.tracked_person_box = person_boxes[person_number-1]
                self.tracked_person_image = person_images[person_number-1]
            
            

    def detector_callback(self,image,detected_array):
        key = self.getKey()
        if self.key == 'f':
            self.tracking = False
            self.key = False
            pub_image = rospy.Publisher("selected_person_image",Image, queue_size=0)
	    pub_box = rospy.Publisher("selected_person_box",TrackedPersonArray, queue_size=0)
            try:
                image = self.bridge.imgmsg_to_cv2(image, "bgr8")
            except CvBridgeError as e:
                print(e)

            self.image_height = image.shape[0]
            self.image_width = image.shape[1]
            self.get_person(image,detected_array)

            if self.tracking == True:
                cv2.circle(self.tracked_person_image,(int((self.tracked_person_box[0]+self.tracked_person_box[1])/2-self.tracked_person_box[0]),int((self.tracked_person_box[3]+self.tracked_person_box[2])/2-self.tracked_person_box[2])), 50, (0,0,255), thickness=1, lineType=8, shift=0)
                #cv2.imshow('Person',self.tracked_person_image)
                #cv2.waitKey(0)
                #cv2.destroyAllWindows()
                msg_array = TrackedPersonArray()
                msg = TrackedPerson()	
                msg.x_min = self.tracked_person_box[0]
                msg.x_max = self.tracked_person_box[1]
                msg.y_max = self.tracked_person_box[3]
                msg.y_min = self.tracked_person_box[2]

                msg_array.objects.append(msg)
                pub_image.publish(self.bridge.cv2_to_imgmsg(self.tracked_person_image, 'bgr8'))
                pub_box.publish(msg_array)

        
    def run(self):
        rospy.spin()

if __name__=='__main__':

    s = select_person()
    s.run()

