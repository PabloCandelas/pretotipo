#!/usr/bin/env python  

import rospy  
import std_msgs 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

#This class will receive a number and an increment and it will publish the   

# result of adding number+increment in a recursive way.  

class BaseScanSubClass():  

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

        rospy.Subscriber("scan", LaserScan, self.printer_cb)   

        ############ CONSTANTS ################ 
        self.mindist = 0.0
        self.minangledist = 0.0
        self.robot_vel = Twist()
        self.robot_vel.linear.x = 0
        self.robot_vel.angular.z = 0
        #********** INIT NODE **********###  
        r = rospy.Rate(10) #1Hz  
        while not rospy.is_shutdown():  

            self.cmd_vel_pub.publish(self.robot_vel)
            
            r.sleep()  
            
            pass

    def printer_cb(self, msg):  

        ## This function receives a number   
        #print ("range min: " , msg.range_min)
        #print ("range max: " , msg.range_max)
        #print ("seq: " , msg.header.seq)
        #print ("ranges[0]: " , msg.ranges[0])
        
        print("ACTIVITY")
        ranges = msg.ranges
        rangess = list(range(1,361))

        for i in range(0,len(ranges)):
            if ranges[i] == 0:
                rangess[i] = 4
            else:
                rangess[i] = float(ranges[i])
   
        self.mindist = min(rangess)
        print("min dist: " , self.mindist)
        
        self.minangledist = (rangess.index(min(rangess)) * msg.angle_increment) + msg.angle_min
        
        print("min angle: " , self.minangledist)
        
        kamaleon = 10 
        if self.mindist <0.25:
            self.mindist = 0
        self.robot_vel.linear.x = kamaleon*self.mindist
        
        if self.minangledist > 3.13:
            self.minangledist = self.minangledist - 6.265
            
        kanguro = 5 #el profe puso kangular
        self.robot_vel.angular.z = kanguro*self.minangledist
        pass  
        
        print (self.robot_vel.linear.x,self.robot_vel.angular.z)

    def cleanup(self):  

        #This function is called just before finishing the node  

        # You can use it to clean things up before leaving  

        # Example: stop the robot before finishing a node.    
        pass  

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  

    rospy.init_node("base_scan_subscriber", anonymous=True)  

    try:  

        BaseScanSubClass()  

    except:  

        rospy.logfatal("base_scan_subscriber died")  
