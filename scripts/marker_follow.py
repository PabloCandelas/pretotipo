#!/usr/bin/env python  
"""
This program is a ROS node. It follows a detetcted object based on its center position and its radius (pixels).
It has 2 input:
    *Center point from the topic:"center"
    *Radius of the object from the topic:"radius"
The output is:
    *Twist msg with the velocities needed to follow the object publishing them to the topic:"marker_follow"
    
Author:Pablo Candelas 01/june/2020
"""
# libraries
import time
import rospy  
import std_msgs 
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

# Classes
class MarkerVel():  
    
    # Functions
    def __init__(self):  
        """ This function initializes the ROS node "lidar_avoid".
        Subscribes it to the topics: "radius" and "center"".
        Publishing outputs to topic:"marker_follow". """
        
        # Start publisher and subscribers
        self.cmd_vel_pub = rospy.Publisher('marker_follow', Twist, queue_size=1)  
        rospy.Subscriber("radius", Int32, self.rad_cb)   
        rospy.Subscriber("center", Point, self.center_cb)   

        # Variables
        self.robot_vel = Twist()
        self.robot_vel.linear.x = 0
        self.robot_vel.angular.z = 0.0
        self.time = 0
        self.cb = 0
        
        # To adjust the execution rate of the while Loop
        r = rospy.Rate(10) #1Hz  
        
        # Publisher loop, it only publishes while the node is active and if it receives a new mesage in less than 2 seconds
        while not rospy.is_shutdown(): 
            self.time = time.time()
            if self.time-self.cb < 2: 
                print self.robot_vel
                self.cmd_vel_pub.publish(self.robot_vel)
            r.sleep()  
            pass

    def rad_cb(self, msg): 
        """ This function determines the linear velocity based on the input msg that is the radius of an object.
        If the object is small is far away if it is big is near."""
        
        # Save the time when this function is called
        self.cb = time.time() 
        
        # Computing the linear velocity
        k_linear = 5 
        self.rad = msg.data
        self.robot_vel.linear.x = k_linear / (self.rad + 0.00001)
        
        # if the object is in the range stop
        if 85 > self.rad > 75:
            self.robot_vel.linear.x = 0
            print("Object in desired distance")
        
        #if the object is closer than it should go back
        if self.rad > 90:
            self.robot_vel.linear.x = -0.1
            print("Avoiding crashing with object")
        
        pass   
        
    def center_cb(self, msg):  
        """ This function determines the angular velocity based on the input msg that is the center of an object.
        It only uses the x coordinate to rotate."""
        
        # Compute the angular velocity
        k_angular = 0.003 
        
        # The middle of a regular frame is 320 pixels so from 0 to 320 is one side and form 320 to 640 is the other
        self.x0 = 320
        self.center_x = msg.x
        self.xdiff = self.x0 - self.center_x
        self.robot_vel.angular.z = k_angular*self.xdiff
        
        pass   

# Main program initializing node and calls "MarkerVel()" class
if __name__ == "__main__":  

    rospy.init_node("marker_follow", anonymous=False)  
    MarkerVel()  

 
