#!/usr/bin/env python
"""
This program is a ROS node. It detects markers from aruco library.
It has one input:
    *Images from the topic:"/wu/image_raw"
The outputs are:
    *The center of the marker in the video (pixels) publishing it to the topic:"center"
    *The radius of the circumscribed circle around the marker (pixels) publishing it to the topic:"radius"
    
Author:Pablo Candelas 01/june/2020
"""

# libararies
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time


# Classes
class ArTracker():

    # Functions
    def __init__(self):
        """ This function initializes the ROS node "ar_tracker".
        Subscribes it to the topic "/wu/image_raw".
        Publishing outputs to topics: "center" and "radius".
        """
        # Init the ros node
        rospy.init_node("ar_tracker")
        self.pub_center = rospy.Publisher('center', Point, queue_size=10)
        self.pub_radius = rospy.Publisher('radius', Int32, queue_size=10)
        self.image_sub = rospy.Subscriber("/wu/image_raw",Image,self.camera_callback)
        
        # Variables
        self.bridge_object = CvBridge() #Creates the bridge object between ROS and opencv images
        self.center_ros = Point()
        self.radius_ros= 0
        self.publish = True
        
        # Constants 
        self.AXISX = 0
        self.AXISY= 1
        self.DEBUG = True
        self.blah = 1
        
        # To adjust the execution rate of the while Loop
        ros_rate = rospy.Rate(10) #10Hz
        
        # Confirmation msg
        print ("Ar_tracker_node started")
        
        # Publisher loop, it only publishes while the node is active and detecting a marker
        while not rospy.is_shutdown():
            if self.publish: 
                self.pub_center.publish(self.center_ros)
                self.pub_radius.publish(self.radius_ros)
            ros_rate.sleep()

        # close all windows
        cv2.destroyAllWindows()
        self.out.release()
    
    def esquinas(self,markerCorners,corners,axis):
        """This function receives the array of arrays "markerCorners" and the "axis" and it returns the array "corners" 
        only with the values from the desired axis from "markerCorners". """
        for i in range(0,4):
            corners[i] = markerCorners[0][0][i][axis]

    def camera_callback(self, data):  
        print("callback")
        self.publish = False
        try:
            # We select bgr8 because its the OpenCV encoding by default
            self.frame = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            
        
        if self.blah == 1:
            # Define the codec and create VideoWriter object.The output is stored in 'outpy.avi' file.
            self.out = cv2.VideoWriter('outnode.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 10, (640,480))
            self.blah = 2
            
        # DETECT THE MARKER
        # Load the dictionary that was used to generate the markers.
        dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        # Initialize the detector parameters using default values
        parameters =  cv2.aruco.DetectorParameters_create()
        # Detect the markers in the image
        markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(self.frame, dictionary, parameters=parameters)   
          
        # If there was at least one marker detected
        if markerIds >0:
            self.publish = True
            # Draw a rectangle around the detected marker
            cv2.aruco.drawDetectedMarkers(self.frame, markerCorners, markerIds)  
                  
            # Computing the center and radius
            xcorners = [0,0,0,0]
            self.esquinas(markerCorners,xcorners,self.AXISX)
            ycorners = [0,0,0,0]
            self.esquinas(markerCorners,ycorners,self.AXISY) 
            """ To calculate the center we know that the marker has a quadrangular shape
            # x coordinates will be the avarage of the minimun value and the maximun value of the corners in the "x" axis
            # y coordinates will be the avarage of the minimun value and the maximun value of the corners in the "y" axis """
            center =int((min(xcorners) + max(xcorners))/2) ,int((min(ycorners) + max(ycorners))/2)
            # The radious is computed used the distance from the center of the marker to any of its corners
            radius = np.sqrt((xcorners[0]-center[0])*(xcorners[0]-center[0]) + (ycorners[0]-center[1])*(ycorners[0]-center[1]))
            # Print the values of the variables "center" and "radius" 
            # Print the values of the variables "center" and "radius" 
            if self.DEBUG: 
                print("----------")
                print("center:")
                print(center) 
                print("radius:")  
                print(radius)  
                
            # Draw the circle thar circumscribes the marker
            cv2.circle(self.frame, (int(center[0]), int(center[1])), int(radius),
            (0, 128, 255), 2)
            # Draw the center with a small circle
            cv2.circle(self.frame, center, 5, (255, 255, 0), -1)       
            
            # Saving the computed values into the ROS publishinf variables
            self.center_ros.x=float(center[0])
            self.center_ros.y=float(center[1])
            self.center_ros.z=0 #As it is an image z is not used.
            self.radius_ros=int(radius)
            
        #cv2.imshow('Test Frame', self.frame)
        # Save the frame for the recorded video
        self.out.write(self.frame)

# Main program only calls the class "ArTracker()"
if __name__ == '__main__':
    ArTracker()
