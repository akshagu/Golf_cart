#! /usr/bin/python

import sys
import time
import logging
import argparse
import rospy
import std_msgs.msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import Header
import numpy as np
import cv2

#from mobilenet_detect.msg import cvbox
#from mobilenet_detect.msg import cvboxarray

current_box = np.zeros(4)
box = np.zeros(4)
box[0] = 300
box[1] = 500
box[2] = 200
box[3] = 400

pred_size = 5
pred = np.zeros((pred_size,len(box)))
pred_x1 = np.random.normal(box[0],50,pred_size)
pred_x2 = np.random.normal(box[1],50,pred_size)
pred_y1 = np.random.normal(box[2],50,pred_size)
pred_y2 = np.random.normal(box[3],50,pred_size)

for i in range(pred_size):
	pred[i,0] = pred_x1[i]
	pred[i,1] = pred_x2[i]
	pred[i,2] = pred_y1[i]
	pred[i,3] = pred_y2[i]

new_boxes = np.zeros((3,4))
new_boxes[0,0] = 0
new_boxes[0,1] = 100
new_boxes[0,2] = 0
new_boxes[0,3] = 100

new_boxes[1,0] = 100
new_boxes[1,1] = 200
new_boxes[1,2] = 100
new_boxes[1,3] = 200

new_boxes[2,0] = 200
new_boxes[2,1] = 300
new_boxes[2,2] = 200
new_boxes[2,3] = 300

pred_diff = np.zeros((len(new_boxes),pred_size,4))

#Calculate differences between the predictions and each of the observed boxes
print(pred)
for i in range (len(new_boxes)):
	for j in range(pred_size):
		for k in range(4):
			pred_diff[i,j,k] = abs(pred[j,k]-new_boxes[i,k])

#Sum the differences for each observed box/prediction pair
print(pred_diff)
diff_sum = np.sum(pred_diff,2)
print(diff_sum)

#An array that for each prediction, provides the bounding box that is the closet fit
best_box_per_pred = np.argmin(diff_sum, axis=0)
counts = np.bincount(best_box_per_pred)
best_box_overall = np.argmax(counts)

print(best_box_overall)

current_box[0] = new_boxes[best_box_overall,0]
current_box[1] = new_boxes[best_box_overall,1]
current_box[2] = new_boxes[best_box_overall,2]
current_box[3] = new_boxes[best_box_overall,3]

print(current_box)


#BELIEF OVER WHICH BOUNDING BOX IS THE PERSON WE WANT TO TRACK (EACH BOUNDING BOX IS A STATE)
#belief_vector = np.ones(len(box))

#for i in range(pred_size):
#	np.sum

