#!/usr/bin/env python  
"""
This program is a ROS node. It avoids objects detected with a lidar.
It has one input:
    *LaserScan msg received from the topic:"/scan"
The output is:
    *Twist msg with the velocities needed to avoid crashes publishing them to the topic:"lidar_avoid"
    
Author:Pablo Candelas 01/june/2020
"""

# Libraries
import rospy  
import std_msgs 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# Classes
class ScanClass(): 
    
    # Functions 
    def __init__(self):  
        """ This function initializes the ROS node "lidar_avoid".
        Subscribes it to the topic "/scan".
        Publishing outputs to topic:"lidar_avoid". """      
        print ("lidar_avoid node started")  
        
        # Start publisher and subscriber
        self.cmd_vel_pub = rospy.Publisher('lidar_avoid', Twist, queue_size=1)  
        rospy.Subscriber("scan", LaserScan, self.esquivar_cb)   

        # Variables
        self.mindist = 0.0
        self.minangledist = 0.0
        self.robot_vel = Twist()
        self.robot_vel.linear.x = 0
        self.robot_vel.angular.z = 0
        self.publish = True
        
        # To adjust the execution rate of the while Loop
        r = rospy.Rate(10) #10Hz  
        
        # Publisher loop, it only publishes while the node is active and trying to avoid an object
        while not rospy.is_shutdown():  
            if self.publish:
                self.cmd_vel_pub.publish(self.robot_vel)
            r.sleep()      
            pass

    def esquivar_cb(self, msg): 
        """ This function initializes avoids the nearest object, it receives 
        the LaserScan data into the variable "msg" """ 
        # Initialize self.publish in False so that in will only be True when it gets into one of the avoiding cases
        self.publish = False
        
        # Acknowledge msg
        print("esquivando")
        
        #Save the incoming tuple into the variable "ranges"
        ranges = msg.ranges
        
        """ Create a list so that we can change the values of the tuple ranges
         This is made because the lidar used returns 0.0 when it receives a value larger than 
         its 3.5m scanning range and that affects the measurements. """
        rangess = list(range(1,361))
        
        # Copy every value from "ranges" into "rangess" and change the 0.0 to 4 if theres any
        for i in range(0,len(ranges)):
            if ranges[i] == 0:
                rangess[i] = 4
            else:
                rangess[i] = float(ranges[i])
                
        # Divide the measurements by side, this lidar begins pointing to the front and makes 360 measurements following a positive turn
        front1 = rangess[0:45]
        front2 = rangess[315:360]
        front = front1 + front2
        left = rangess[45:135]
        back = rangess[135:225]
        right = rangess[225:315]
        
        # The order of the followinf if's is important
        # if there is no object near go forward
        if min(rangess) >= 0.3:
            self.robot_vel.angular.z = 0
            self.robot_vel.linear.x = 0.2

        # From now on an object is detected so the output will be published
        # if there is an object in front go back and turn to the best side to avoid it
        if min(front) < 0.25:
            self.publish = True
            print ("front")
            print(min(front))
            self.robot_vel.linear.x = -0.1
            if front.index(min(front)) > 45:
                self.robot_vel.angular.z = 0.3
                
            if front.index(min(front)) <= 45:
                self.robot_vel.angular.z = -0.3            
                
        # if there is an object on the right turn to the left    
        if min(right) < 0.2:
            self.publish = True
            print ("right")
            print(min(right))
            self.robot_vel.angular.z = 0.3
            
        # if there is an object in the back go front and turn to the best side to avoid it  
        if min(back) < 0.2:
            self.publish = True
            print ("back")
            print(min(back))
            self.robot_vel.linear.x = 0.1
            if min(left) > min(right):
                self.robot_vel.angular.z = 0.3
                
            if min(left) < min(right):
                self.robot_vel.angular.z = -0.3
                
        # if there is an object on the left turn to the right
        if min(left) < 0.2:
            self.publish = True
            print ("left")
            print(min(left))
            self.robot_vel.angular.z = -0.3

        # print the minimun distance detected to corroborate the decision  
        self.mindist = min(rangess)
        print("min dist: " , self.mindist)
        

# Main program initializing node and calls "ScanClass()"
if __name__ == "__main__":  
    rospy.init_node("lidar_avoid", anonymous=False)  
    try:  
        ScanClass()  
    except:  
        rospy.logfatal("esquivador died")
