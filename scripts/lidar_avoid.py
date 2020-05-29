#!/usr/bin/env python  

import rospy  
import std_msgs 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

#This class will receive a number and an increment and it will publish the   

# result of adding number+increment in a recursive way.  

class ScanClass():  

    def __init__(self):  

        rospy.on_shutdown(self.cleanup)  

        print "Node started..."  
        
        ###******* INIT PUBLISHERS *******###  

        print "Setting publisher..."  

        ##  pub = rospy.Publisher('setPoint', UInt16MultiArray, queue_size=1)  

        self.cmd_vel_pub = rospy.Publisher('lidar_avoid', Twist, queue_size=1)  

        print "Publishers ok"  

        print "Starting Node..."  

        ############################### SUBSCRIBERS #####################################  

        rospy.Subscriber("scan", LaserScan, self.esquivar_cb)   

        ############ CONSTANTS ################ 
        self.mindist = 0.0
        self.minangledist = 0.0
        self.robot_vel = Twist()
        self.robot_vel.linear.x = 0
        self.robot_vel.angular.z = 0
        self.publicar = True
        #********** INIT NODE **********###  
        r = rospy.Rate(10) #1Hz  
        while not rospy.is_shutdown():  
            if self.publicar:
                self.cmd_vel_pub.publish(self.robot_vel)
            
            r.sleep()  
            
            pass

    def esquivar_cb(self, msg):  
        self.publicar = False
        print("esquivando")
        ranges = msg.ranges
        rangess = list(range(1,361))
        for i in range(0,len(ranges)):
            if ranges[i] == 0:
                rangess[i] = 4
            else:
                rangess[i] = float(ranges[i])
        front1 = rangess[0:45]
        front2 = rangess[315:360]
        front = front1 + front2
        left = rangess[45:135]
        back = rangess[135:225]
        right = rangess[225:315]
        
        #el orden en que estan acomodados los "if" siguientes es importante
        
        
        """if 0.3 < min(front) < 0.5:
            self.publicar = True
            print("casi")
            if front.index(min(front)) > 45:
                self.robot_vel.angular.z = 0.3
                
            if front.index(min(front)) <= 45:
                self.robot_vel.angular.z = -0.3 """
                
        """if min(left) > min(right):
                self.robot_vel.angular.z = 0.8
                
            if min(left) < min(right):
                self.robot_vel.angular.z = -0.8""" 
                
        if min(front) < 0.25:
            self.publicar = True
            print ("front")
            print(min(front))
            self.robot_vel.linear.x = -0.1
            if front.index(min(front)) > 45:
                self.robot_vel.angular.z = 0.3
                
            if front.index(min(front)) <= 45:
                self.robot_vel.angular.z = -0.3
            """if min(left) > min(right):
                self.robot_vel.angular.z = 0.5
                
            if min(left) < min(right):
                self.robot_vel.angular.z = -0.5"""
            
            
        if min(right) < 0.2:
            self.publicar = True
            print ("right")
            print(min(right))
            self.robot_vel.angular.z = 0.3
            
        if min(back) < 0.2:
            self.publicar = True
            print ("back")
            print(min(back))
            self.robot_vel.linear.x = 0.1
            if min(left) > min(right):
                self.robot_vel.angular.z = 0.3
                
            if min(left) < min(right):
                self.robot_vel.angular.z = -0.3
            
        if min(left) < 0.2:
            self.publicar = True
            print ("left")
            print(min(left))
            self.robot_vel.angular.z = -0.3
        
        
            
           
        self.mindist = min(rangess)
        print("min dist: " , self.mindist)
        """ 
        self.minangledist = (rangess.index(min(rangess)) * msg.angle_increment) + msg.angle_min
        
        print("min angle: " , self.minangledist)"""
        
        """
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
        """
    def cleanup(self):  

        #This function is called just before finishing the node  

        # You can use it to clean things up before leaving  

        # Example: stop the robot before finishing a node.    
        pass  

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  

    rospy.init_node("lidar_avoid", anonymous=False)  

    try:  

        ScanClass()  

    except:  

        rospy.logfatal("esquivador died")
