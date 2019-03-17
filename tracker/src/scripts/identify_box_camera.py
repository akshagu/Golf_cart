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

#from dnn_detect.msg import DetectedObject
#from dnn_detect.msg import DetectedObjectArray

from mobilenet_detect.msg import cvbox
from mobilenet_detect.msg import cvboxarray

from tracker.msg import TrackedPerson
from tracker.msg import TrackedPersonArray

from message_filters import ApproximateTimeSynchronizer, TimeSynchronizer, Subscriber

bridge = CvBridge()
current_box = np.zeros(8)
cluster_orig = np.zeros(3)
time_count = 0

pub_image = rospy.Publisher("tracked_box_image",Image, queue_size=1)
pub_box = rospy.Publisher("tracked_box",TrackedPersonArray, queue_size=1)

def predictions(last_box,pred_size,cluster_orig):
	percent_var = 10 #Percent change from the box characteristics that we use in our prediction standard deviations
	pred = np.zeros((pred_size,len(last_box)))
	pred_x_min = np.random.normal(last_box[0],last_box[0]*percent_var/100,pred_size)
	pred_x_max = np.random.normal(last_box[1],last_box[1]*percent_var/100,pred_size)
	pred_y_min = np.random.normal(last_box[2],last_box[2]*percent_var/100,pred_size)
	pred_y_max = np.random.normal(last_box[3],last_box[3]*percent_var/100,pred_size)
	pred_area = np.random.normal(last_box[4],last_box[4]*percent_var/100,pred_size)
	pred_cluster_1 = np.random.normal(cluster_orig[0],cluster_orig[0]*percent_var/100,pred_size)
	pred_cluster_2 = np.random.normal(cluster_orig[1],cluster_orig[1]*percent_var/100,pred_size)
	pred_cluster_3 = np.random.normal(cluster_orig[2],cluster_orig[2]*percent_var/100,pred_size)

	for i in range(pred_size):
		pred[i,0] = pred_x_min[i]
		pred[i,1] = pred_x_max[i]
		pred[i,2] = pred_y_min[i]
		pred[i,3] = pred_y_max[i]
		pred[i,4] = pred_area[i]
		pred[i,5] = pred_cluster_1[i]
		pred[i,6] = pred_cluster_2[i]
		pred[i,7] = pred_cluster_3[i]
	return pred

def best_matched_box(person_boxes,pred):
	pred_diff = np.zeros((len(person_boxes),len(pred),8))

	#Calculate differences between the predictions and each of the observed boxes
	for i in range (len(person_boxes)):
		for j in range(len(pred)):
			for k in range(8):
				pred_diff[i,j,k] = abs(pred[j,k]-person_boxes[i,k])
				#Below we weight the value of the color differences higher than the other prediction differences
				if k > 4:
					pred_diff[i,j,k] = 100000*pred_diff[i,j,k]

	#Sum the differences for each observed box/prediction pair
	diff_sum = np.sum(pred_diff,2)

	#An array that for each prediction, provides the bounding box that is the closet fit
	best_box_per_pred = np.argmin(diff_sum, axis=0)
	counts = np.bincount(best_box_per_pred)
	best_box_overall = np.argmax(counts)


	current_box[0] = person_boxes[best_box_overall,0]
	current_box[1] = person_boxes[best_box_overall,1]
	current_box[2] = person_boxes[best_box_overall,2]
	current_box[3] = person_boxes[best_box_overall,3]
	current_box[4] = person_boxes[best_box_overall,4]
	current_box[5] = person_boxes[best_box_overall,5]
	current_box[6] = person_boxes[best_box_overall,6]
	current_box[7] = person_boxes[best_box_overall,7]

	return current_box


def callback(image,detected_array):
	pub_image = rospy.Publisher("tracked_box_image",Image, queue_size=0)
	pub_box = rospy.Publisher("tracked_box",TrackedPersonArray, queue_size=0)

	image_width = 1280
	image_height = 720
	#Create empty array that will containg bounding box coordinates for detected people
	global time_count
	global current_box
	global cluster_orig

	#assert image.header.stamp == detected_array.header.stamp

	try:
		cv_image = bridge.imgmsg_to_cv2(image, "bgr8")
	except CvBridgeError as e:
		print(e)	
	person_count = 0
	for i in range(len(detected_array.objects)):
		if detected_array.objects[i].class_name == 'person':
			person_count += 1

	person_boxes = np.zeros((person_count,8))
	if person_count > 0:
		person_count = 0
		for i in range(len(detected_array.objects)):
			if detected_array.objects[i].class_name == 'person':
				coordinates = np.zeros(8)
				coordinates[0] = detected_array.objects[i].x_min
				if coordinates[0] < 0:
					coordinates[0] = 0
				coordinates[1] = detected_array.objects[i].x_max
				if coordinates[1] > image_width:
					coordinates[1] = image_width
				coordinates[2] = detected_array.objects[i].y_min
				if coordinates[2] < 0:
					coordinates[2] = 0
				coordinates[3] = detected_array.objects[i].y_max
				if coordinates[3] > image_height:
					coordinates[3] = image_height

				#coordinate 4 is the area of the bounding box
				coordinates[4] = abs(coordinates[1]-coordinates[0])*abs(coordinates[3]-coordinates[2])
				#K means clustering to get average color
				this_box_image = cv_image[int(coordinates[2]):int(coordinates[3]), int(coordinates[0]):int(coordinates[1])]
                                Z = this_box_image.reshape((-1,3))
				Z = np.float32(Z)
                                if Z.size != 0:


				        # define criteria, number of clusters(K) and apply kmeans()
			  	        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
				        K = 1
				        ret,label,center=cv2.kmeans(Z,K,None,criteria,10,cv2.KMEANS_RANDOM_CENTERS)
				        #Coordinates 5, 6, and 7 correspond to the average color
				        coordinates[5], coordinates[6], coordinates[7] = center[0,0], center[0,1], center[0,2]

				        person_boxes[person_count] = coordinates
				        person_count += 1
	
		#If this is the first time we're doing the callback, track the first person in the image
		if time_count == 0:
			time_count += 1
			last_box = person_boxes[0]
			cluster_orig[0], cluster_orig[1], cluster_orig[2] = last_box[5], last_box[6], last_box[7]
		else:
			last_box = current_box

		pred_size = 5 #Number of predictions to make from last state bounding box
		pred = predictions(last_box,pred_size,cluster_orig)
		current_box = best_matched_box(person_boxes,pred)

		if person_count > 0:
			cv2.rectangle(cv_image,(int(current_box[0]),int(current_box[2])),(int(current_box[1]),int(current_box[3])),(0,255,0),3)
	else:
		if time_count > 0:
			last_box = current_box
		else:
			last_box = np.zeros(8)
	
	msg_array = TrackedPersonArray()
	msg = TrackedPerson()	
	msg.x_min = current_box[0]
	msg.x_max = current_box[1]
	msg.y_max = current_box[2]
	msg.y_min = current_box[3]

	msg_array.objects.append(msg)
	pub_image.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
	pub_box.publish(msg_array)
        

	#cv2.imshow('image',cv_image)
	#cv2.waitKey(200)
	#cv2.destroyAllWindows()

	

def track():


	rospy.init_node('tracked_person', anonymous=False)
	
	rate = rospy.Rate(5) # 5hz

	image_sub = Subscriber("/v4l/camera/image_raw",Image,queue_size=1)
	detected_array_sub = Subscriber('cv_test_box',cvboxarray,queue_size=1)
	ats = ApproximateTimeSynchronizer([image_sub, detected_array_sub], 5,.4)
	#tss = TimeSynchronizer(Subscriber("/usb_cam/image_raw",Image), Subscriber('/dnn_objects',DetectedObjectArray))
	ats.registerCallback(callback)

	rospy.spin()	

if __name__ == '__main__':
	track()


