#!/usr/bin/python

"""
Created on Thu Mar 21 17:07:35 2019

@author: ivorbach
"""

import sys
import time
import logging
import argparse
import rospy
import std_msgs.msg
import random
import geometry_msgs.msg
from math import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import Header
import numpy as np
import cv2

from tracker.msg import TrackedPerson
from tracker.msg import TrackedPersonArray


class particle_filter:
    
    def __init__(self):
        rospy.init_node('tracked_person', anonymous=False)
        self.bridge = CvBridge()
	
        #Initialize total number of particles
        self.N = 1000

        #Initialize states as false
        self.follow_started = False
        self.new_person = False

        #rate = rospy.Rate(5) # 5hz
        rospy.Subscriber("selected_person_box",TrackedPersonArray,self.new_person_callback,queue_size=1)
        rospy.Subscriber("/v4l/camera/image_raw",Image,self.camera_callback,queue_size=1)
            
        
    def generate_particles(self,image,start_range,end_range,mu_x,mu_y):
        self.pixel_distance_sigma = 40
        for i in range(start_range,end_range):
            self.particles[i,0] = np.absolute(int(np.random.normal(mu_x, self.pixel_distance_sigma)))
            self.particles[i,1] = np.absolute(int(np.random.normal(mu_y, self.pixel_distance_sigma)))

            #Make sure pixel coordinates are not outside the bounds of the image
            if self.particles[i,0] < 0:
                self.particles[i,0] = 0
            if self.particles[i,0] > self.image_width-1:
                self.particles[i,0] = self.image_width-1
            if self.particles[i,1] < 0:
                self.particles[i,1] = 0
            if self.particles[i,1] > self.image_height-1:
                self.particles[i,1] = self.image_height-1
            self.particles[i,2:5] = image[int(self.particles[i,1])][int(self.particles[i,0])]

    def new_person_callback(self,detected_array):
        self.follow_started = True
        self.new_person = True
        self.orig_pixel_x = int((detected_array.objects[0].x_min+detected_array.objects[0].x_max)/2)
        self.orig_pixel_y = int((detected_array.objects[0].y_min+detected_array.objects[0].y_max)/2)

        
    # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
    def Gaussian(self, mu, sigma, x):
        return np.exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / np.sqrt(2.0 * np.pi * (sigma ** 2))
    
    # calculates how likely a color measurement should be        
    def measurement_prob(self, curr_pixel_colors):
        #This pixel_sigma is a hyperparameter that can be tuned
        pixel_sigma = 5
        #prob = 1.0;
        color_dist = 0.0
        for i in range(len(curr_pixel_colors)):
            #prob *= self.Gaussian(self.orig_pixel_color[i], pixel_sigma, curr_pixel_colors[i])
            #print(i,'i value')
            #print(self.orig_pixel_color[i],'source pixel color')
            #print(curr_pixel_colors[i],'current pixel color')
            color_dist += (self.orig_pixel_color[i]-curr_pixel_colors[i])**2
        prob = 1.0/color_dist
        return prob
        
    def create_next_particle_set(self,image):
        #Percentage of particles selected from last set
        percent_carry = 0.2

        new_particle_set = np.zeros((self.N,6))
        index = random.randint(0,self.N-1)
        beta = 0.0
        max_weight = max(self.weights)

        #The first *percent_carry* percentage of the new particle set will be selected from the old set
        for i in range(int(self.N*percent_carry)):
            beta += random.uniform(0,2.0*max_weight)
            #while self.particles[index,5] < beta:
            while self.weights[index] < beta:
                #beta -= self.particles[index,5]
                beta -= self.weights[index]
                index +=1
                if index > self.N-1:
                    index = 0
            new_particle_set[i,0:2] = self.particles[index,0:2]
            new_particle_set[i,2:5] = image[int(new_particle_set[i,1])][int(new_particle_set[i,0])]

        #The remaining set of the new particles will be generated randomly from that first carried over group
        self.particles = new_particle_set

        for i in range(int(self.N*percent_carry)):
             if i == 0:
                 start = int(self.N*percent_carry)
                 end = start + int(1/percent_carry)-2
             else:
                 start = end + 1
                 end = start + int(1/percent_carry)-2
             self.generate_particles(image,start,end+1,self.particles[i,0],self.particles[i,1])
    
    def camera_callback(self, picture_image):
        pub_image = rospy.Publisher("particles_on_image",Image, queue_size=0)
	#pub_box = rospy.Publisher("particle_center",Pose2D, queue_size=0)
        try:
            image = self.bridge.imgmsg_to_cv2(picture_image, "bgr8")
        except CvBridgeError as e:
            print(e)
            
        self.image_height = image.shape[0]
        self.image_width = image.shape[1]
        
        #If we have at least once selected a person to follow
        if self.follow_started == True:

            #If a new person is being assinged for tracking, create a new set of particles and weights around the new person
            if self.new_person == True:
                self.new_person = False
                self.orig_pixel_color = image[self.orig_pixel_y][self.orig_pixel_x]
                self.particles = np.zeros((self.N,6)).astype(int)
                self.generate_particles(image,0,self.N,self.orig_pixel_x,self.orig_pixel_y)
            else:     
                #Determine pixel color weighting likelihood of curret particle set
                self.weights = []
                for i in range(self.N):
                    #self.particles[i,5] = self.measurement_prob(self.particles[i,2:5])
                    self.weights.append(self.measurement_prob(self.particles[i,2:5]))

                #Select next set of particles
                self.create_next_particle_set(image)
        
            #particles_avg = []

            #Draw particles onto image for visualization
            for i in range(self.N):
                cv2.circle(image,(int(self.particles[i,0]),int(self.particles[i,1])), 5, (255,255,0), thickness=1, lineType=8, shift=0)

            #Publish
            pub_image.publish(self.bridge.cv2_to_imgmsg(image, 'bgr8'))

            
            
        

        
            
            
            
            
    #def particle_callback(self,image,detected_array):
        
            
        #NEED TO CHANGE BELOW TO PUB IMAGE WITH PARTICLES ON THEM, NOT A BOX
        
        #pub_particle_avg = rospy.Publisher("tracked_box",TrackedPersonArray, queue_size=0)
        

    



    def run(self):
        rospy.spin()
        
if __name__=='__main__':
    p = particle_filter()
    p.run()
        
