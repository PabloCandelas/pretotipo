"""
This program is a ROS node. It detects markers from aruco library.
It has one input:
    *Images from the topic:"/wu/image_raw"
The outputs are:
    *The center of the marker in the video (pixels) to the topic:"center"
    *The radius of the circumscribed circle around the marker (pixels) to the topic:"radius"
    
Author:Pablo Candelas 01/june/2020
"""
#!/usr/bin/env python

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


# +
class ArTracker():
    def __init__(self):
        # Init the ros node
        rospy.init_node("ar_tracker")
        self.pub_center = rospy.Publisher('center', Point, queue_size=10)
        self.pub_radius = rospy.Publisher('radius', Int32, queue_size=10)
        self.image_sub = rospy.Subscriber("/wu/image_raw",Image,self.camera_callback)
        self.bridge_object = CvBridge() #Creates the bridge object between ROS and opencv images
        self.center_ros = Point()
        self.radius_ros=0
        self.EJEX = 0
        self.EJEY=1
        self.publish = True
        
        #To adjust the execution rate of the while Loop
        ros_rate = rospy.Rate(10) #10Hz
        # keep looping
        print ("Iniciado")
        while not rospy.is_shutdown():
            if self.publish: 
                self.pub_center.publish(self.center_ros)
                self.pub_radius.publish(self.radius_ros)
            ros_rate.sleep()

        # close all windows
        cv2.destroyAllWindows()
    
    def esquinas(self,markerCorners,esq,eje):
        for i in range(0,4):
            esq[i] = markerCorners[0][0][i][eje]

    def camera_callback(self, data):  
        print("callback")
        self.publish = False
        try:
            # We select bgr8 because its the OpenCV encoding by default
            self.frame = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        #DETECTAR MARCADOR
        #Load the dictionary that was used to generate the markers.
        dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

        # Initialize the detector parameters using default values
        parameters =  cv2.aruco.DetectorParameters_create()
        
        # Detect the markers in the image
        markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(self.frame, dictionary, parameters=parameters)        
        #Si se detecto un marcador
        if markerIds >0:
            self.publish = True
            #dibujar cuadro alrededor
            cv2.aruco.drawDetectedMarkers(self.frame, markerCorners, markerIds)
            
            #Caluclo de radio y centro
            esqx = [0,0,0,0]
            self.esquinas(markerCorners,esqx,self.EJEX)
            esqy = [0,0,0,0]
            self.esquinas(markerCorners,esqy,self.EJEY)
               
            center =int((min(esqx) + max(esqx))/2) ,int((min(esqy) + max(esqy))/2)
            radius = np.sqrt((esqx[0]-center[0])*(esqx[0]-center[0]) + (esqy[0]-center[1])*(esqy[0]-center[1]))
            print("----------")
            print("center:")
            print(center) 
            print("radius:")  
            print(radius)  
            #Dibujar circulo y centro
            cv2.circle(self.frame, (int(center[0]), int(center[1])), int(radius),(0, 128, 255), 2)
            cv2.circle(self.frame, center, 5, (255, 255, 0), -1)     
            #mostrar en pantalla     
       
            self.center_ros.x=float(center[0])
            self.center_ros.y=float(center[1])
            self.center_ros.z=0 #As it is an image z is not used.
            self.radius_ros=int(radius)
        #cv2.imshow('Test Frame', self.frame)
        key = cv2.waitKey(1) & 0xFF

# Main program only calling the class "ArTracker()"
if __name__ == '__main__':
    ArTracker()
