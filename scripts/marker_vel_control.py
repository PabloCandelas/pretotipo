#!/usr/bin/env python  

import rospy  
import std_msgs 
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

#This class follows a color object

class MarkerVel():  

    def __init__(self):  

        rospy.on_shutdown(self.cleanup)  

        print "Node started..."  
        
        
        ###******* INIT PUBLISHERS *******###  

        print "Setting publisher..."  

        ##  pub = rospy.Publisher('setPoint', UInt16MultiArray, queue_size=1)  

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)  

        print "Publishers ok"  

        print "Starting Node..."  

        ############################### SUBSCRIBERS #####################################  

        rospy.Subscriber("radius", Int32, self.rad_cb)   
        rospy.Subscriber("center", Point, self.center_cb)   

        ############ CONSTANTS ################ 
        self.robot_vel = Twist()
        self.robot_vel.linear.x = 0
        self.robot_vel.angular.z = 0.0
        
        #********** INIT NODE **********###  
        r = rospy.Rate(10) #1Hz  
        while not rospy.is_shutdown():  
            print self.robot_vel
            self.cmd_vel_pub.publish(self.robot_vel)

            r.sleep()  
            
            pass

    def rad_cb(self, msg):  
        kamaleon = 10 #el profe puso klinear
        self.rad = msg.data
        self.robot_vel.linear.x = kamaleon / (self.rad + 0.00001)
        if 85 > self.rad > 75:
            self.robot_vel.linear.x = 0
            print("cerca")
        if self.rad > 90:
            self.robot_vel.linear.x = -0.1
            print("muy cerca")
        
        pass   
        
    def center_cb(self, msg):  
        kanguro = 0.004 #el profe puso kangular
        self.x0 = 320
        self.center_x = msg.x
        self.xdiff = self.x0 - self.center_x
        self.robot_vel.angular.z = kanguro*self.xdiff
        pass   
        

    def cleanup(self):  

        #This function is called just before finishing the node  

        # You can use it to clean things up before leaving  

        # Example: stop the robot before finishing a node.    
        pass  

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  

    rospy.init_node("marker_vel_control", anonymous=False)  
    MarkerVel()  

 
