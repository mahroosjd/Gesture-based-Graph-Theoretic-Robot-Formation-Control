#!/usr/bin/env python
import rospy

import numpy as np

import math

import time

from std_msgs.msg import UInt8

from marvelmind_nav.msg import hedge_pos_ang

from geometry_msgs.msg import Twist

from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion

from kobuki_msgs.msg import BumperEvent

kp = 0.4
kp1 = 0.005
#kp1 = 0.008    #testing
#kp1 = 0.006
#kp1 = 0.01
kp2 = 0.01


roll = pitch = yaw = 0.0

x_coord_1 = 0
y_coord_1 = 0
x_coord2 = 0
y_coord2 = 0
x_coord3 = 0
y_coord3 = 0
x_coord4 = 0
y_coord4 = 0

bump = 0
target_ang = 0 

num_open_fingers = 0

def get_pos(mes):
    global x_coord_1, y_coord_1, x_coord2, y_coord2, x_coord3, y_coord3
    if(mes.address == 3):
        x_coord_1 = mes.x_m
        y_coord_1 = mes.y_m

    if(mes.address == 4):
        x_coord2 = mes.x_m
        y_coord2 = mes.y_m

    if(mes.address == 5):
        x_coord3 = mes.x_m
        y_coord3 = mes.y_m
        
    if(mes.address == 6):
        x_coord4 = mes.x_m
        y_coord4 = mes.y_m
    #rospy.loginfo("x coordinate of beacon 1 = %f: y coordinate of beacon 1 = %f",x_coord_1,y_coord_1)

def get_rotation(msg):

    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    #rospy.loginfo("yaw = %f",yaw)
    

def detect_coll(mes):

    global bump
    if(mes.state == 1):
        bump = 1
        rospy.loginfo("Collision detected")
        
def callback(data):

    global num_open_fingers
    
    if (data.data == 1):
        num_open_fingers = 1
    elif (data.data == 2):
        num_open_fingers = 2
    elif (data.data == 3):
        num_open_fingers = 3
    elif (data.data == 4):
        num_open_fingers = 4
    elif (data.data == 5):
        num_open_fingers = 5 

        
#Function corresponding to 1 open finger      
def get_into_form():

    x_target = 0
    y_target = 0

    while ((not rospy.is_shutdown()) and (num_open_fingers==1)):
    
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&---3 ROBOTS---&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
            
#------------Linear (Vertical) Formation with 2 metre distance between each robot (along y)-----------------------
        #x_target = x_coord_1-x_coord_1+0.5*x_coord2+0.5*x_coord3+0.5*2*2-0.5*1*2-0.5*1*2
        #y_target = y_coord_1-y_coord_1+0.5*y_coord2+0.5*y_coord3+0.5*2*2-0.5*1*4-0.5*1*6
#------------------------------------------------------------------------------------------------            
#------------Linear (Vertical) Formation with 1 metre distance between each robot (along y)----------------------
        #x_target = x_coord_1-x_coord_1+0.5*x_coord2+0.5*x_coord3+0.5*2*2-0.5*1*2-0.5*1*2
        #y_target = y_coord_1-y_coord_1+0.5*y_coord2+0.5*y_coord3+0.5*2*2-0.5*1*3-0.5*1*4
#------------------------------------------------------------------------------------------------
#------------Horizontal Formation with 1 metre distance between each robot (along x)-----------------------
        #x_target = x_coord_1-x_coord_1+0.5*x_coord2+0.5*x_coord3+0.5*2*0.5-0.5*1*1.5-0.5*1*2.5
        #y_target = y_coord_1-y_coord_1+0.5*y_coord2+0.5*y_coord3+0.5*2*1-0.5*1*1-0.5*1*1
#------------------------------------------------------------------------------------------------
#------------Triangular Formation (Isosceles)----------------------------------------------------
        #x_target = x_coord_1-x_coord_1+0.5*x_coord2+0.5*x_coord3+0.5*2*1-0.5*1*3-0.5*1*2
        #y_target = y_coord_1-y_coord_1+0.5*y_coord2+0.5*y_coord3+0.5*2*1-0.5*1*1-0.5*1*2
#------------------------------------------------------------------------------------------------

#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&----4 ROBOTS---&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

#-------------Horizontal Formation with 1 metre distance between each robot (along x)-----------
        x_target = x_coord_1 - 0.5*x_coord_1 + 0.5*x_coord3 + 0.5*1*1 - 0.5*1*3
        y_target = y_coord_1 - 0.5*y_coord_1 + 0.5*y_coord3 + 0.5*1*2 - 0.5*1*2
#------------------------------------------------------------------------------------------------


        #x_target = 1   #Testing
        #y_target = 3   #Testing
            
        rospy.loginfo("target_x = %f , target_y = %f",x_target,y_target)
        diff_x = x_target - x_coord_1
        diff_y = y_target - y_coord_1
# Finding the Target Angle
        rospy.loginfo("difference in x:%f difference in y:%f ",diff_x,diff_y)
        target_ang = math.atan2(diff_y,diff_x)
        rospy.loginfo("Target Angle = %f",target_ang)
            
#Angle Reading from /odom (Converted to Euler Angles from Quaternion)
        yaw1 = yaw
        rospy.loginfo("Current Angle = %f", yaw1)
            
        diff_ang1 = target_ang-yaw1
     
            
        if (np.sign(target_ang) == np.sign(yaw1)):
            diff_ang = diff_ang1
                
        elif (np.sign(target_ang) != np.sign(yaw1)) and (abs(diff_ang1) <= math.pi):
            diff_ang = diff_ang1
                
        elif (np.sign(target_ang) != np.sign(yaw1)) and (abs(diff_ang1) > math.pi):
            if (diff_ang1 < 0):
                diff_ang = (2*math.pi)-abs(diff_ang1)
            elif (diff_ang1 > 0):
                diff_ang = (-2*math.pi)+abs(diff_ang1)
                    
        rospy.loginfo("*Adjusted* Angle by which robot will rotate = %f ",diff_ang)
                    
#To avoid discontinuity at cos(pi/2)...since as cosine will begin to decrease near pi/2,  the  linear velocity will increase a lot
        diff_ang2 = diff_ang
        if (1<=diff_ang<=1.57):
            diff_ang2 = 1
        if (1.57<diff_ang<=2.14):  
            diff_ang2 = 2.14
                
        if (-1.57<=diff_ang<=-1):
            diff_ang2 = -1
        if (-2.14<diff_ang<=-1.57): #to avoid discontinuity at cos(-pi/2)
            diff_ang2 = -2.14
                    
        dif_x = x_target-x_coord_1
        dif_y = y_target-y_coord_1
            
        lin_vel = kp1* math.sqrt(math.pow(dif_x,2)+math.pow(dif_y,2))/(0.1*abs(math.cos(diff_ang2)))

        ang_vel = kp2*(diff_ang)/0.1

        rospy.loginfo("Stage 1: Linear Velocity = %f , Angular Velocity = %f",lin_vel,ang_vel)
        command.linear.x = lin_vel
        command.angular.z = ang_vel
            
        pub.publish(command)
        if(bump==1):
            target_obs = yaw+(math.pi/2)# to avoid obstacle, change angle by 90 degrees
            if (target_obs>math.pi):
                target_obs = target_obs-2*(math.pi)
            diff_obs = target_obs-yaw
            while not rospy.is_shutdown():
                diff_obs = target_obs-yaw
                vel.angular.z = kp*(diff_obs)
                pub.publish(vel)
                if abs(diff_obs)<0.09:
                        bump = 0
                        break
                r1.sleep()  
        diff_x = x_target-x_coord_1
        diff_y = x_target-y_coord_1
        x_sq = math.pow(diff_x,2)
        y_sq = math.pow(diff_y,2)
        dist = math.sqrt(x_sq+y_sq)
                
        rospy.loginfo("Distance = %f", dist)
                
        difx_r1_r2 = x_coord_1-x_coord2
        dify_r1_r2 = y_coord_1-y_coord2
        dist_r1_r2 = math.sqrt(math.pow(difx_r1_r2,2)+math.pow(dify_r1_r2,2))
            
        difx_r1_r3 = x_coord_1-x_coord3
        dify_r1_r3 = y_coord_1-y_coord3
        dist_r1_r3 = math.sqrt(math.pow(difx_r1_r3,2)+math.pow(dify_r1_r3,2))
            
        difx_r2_r3 = x_coord2-x_coord3
        dify_r2_r3 = y_coord2-y_coord3
        dist_r2_r3 = math.sqrt(math.pow(difx_r2_r3,2)+math.pow(dify_r2_r3,2))
        
        difx_r3_r4 = x_coord3-x_coord4
        dify_r3_r4 = y_coord3-y_coord4
        dist_r3_r4 = math.sqrt(math.pow(difx_r3_r4,2)+math.pow(dify_r3_r4,2))
        
        difx_r1_r4 = x_coord_1-x_coord4
        dify_r1_r4 = y_coord_1-y_coord4
        dist_r1_r4 = math.sqrt(math.pow(difx_r1_r4,2)+math.pow(dify_r1_r4,2))
        
        difx_r2_r4 = x_coord2-x_coord4
        dify_r2_r4 = y_coord2-y_coord4
        dist_r2_r4 = math.sqrt(math.pow(difx_r2_r4,2)+math.pow(dify_r2_r4,2))
            
        #if (1.9<dist_r1_r2<2.1) and (3.9<dist_r1_r3<4.1) and (1.9<dist_r2_r3<2.1):   #Vertical 2 metre 
        #if (0.9<dist_r1_r2<1.1) and (1.9<dist_r1_r3<2.1) and (0.9<dist_r2_r3<1.1):   #Vertical 1 metre
        #if (0.9<dist_r1_r2<1.1) and (1.9<dist_r1_r3<2.1) and (0.9<dist_r2_r3<1.1):   #Horizontal 1 metre
        #if (1.85<dist_r1_r2<2.15) and (1.25<dist_r1_r3<1.55) and (1.25<dist_r2_r3<1.55): #Triangular

        #if (abs(x_coord2-x_coord_1)<=0.08) and (1.92<=(y_coord2-y_coord_1)<=2.08) and (abs(x_coord3-x_coord_1)<=0.08) and (5.92<=(y_coord3-y_coord_1)<=6.08) and (abs(x_coord3-x_coord_2)<=0.08) and (1.92<=(y_coord3-y_coord_2)<=2.08):
        
        if (0.9<dist_r1_r2<1.1) and (1.9<dist_r1_r3<2.1) and (0.9<dist_r2_r3<1.1) and (0.9<dist_r3_r4<1.1) and(2.9<dist_r1_r4<3.1) and (1.9<dist_r2_r4<2.1):
            rospy.loginfo("All robots in Linear Formation")
            rospy.loginfo("Sum of inter-robot distances: %f", dist_r1_r2+dist_r2_r3+dist_r1_r3)
            num_open_fingers = 0
            break
            
        r.sleep()
        
#Function corresponding to 2 open fingers      
def make_form_move():
    x_target = 0
    y_target = 0
    target_ang = math.pi/2
     
    while ((not rospy.is_shutdown()) and (num_open_fingers==2)):
        yaw1 = yaw
        diff_ang1 = target_ang-yaw1 
              
        if (np.sign(target_ang) == np.sign(yaw1)):
            diff_ang = diff_ang1
                
        elif (np.sign(target_ang) != np.sign(yaw1)) and (abs(diff_ang1) <= math.pi):
            diff_ang = diff_ang1
                
        elif (np.sign(target_ang) != np.sign(yaw1)) and (abs(diff_ang1) > math.pi):
            if (diff_ang1 < 0):
                diff_ang = (2*math.pi)-abs(diff_ang1)
            elif (diff_ang1 > 0):
                diff_ang = (-2*math.pi)+abs(diff_ang1)

        
        vel.linear.x = 0
        vel.angular.z = kp*(diff_ang)
        pub.publish(vel)
        if abs(diff_ang)<0.09:
            break
                       
        r1.sleep()  
#-----------------------------------------------------------------------------------
#NEXT STAGE: Having the formation move linearly while maintaining inter-agent distances
#-----------------------------------------------------------------------------------
    #tic1 = time.perf_counter()
    while ((not rospy.is_shutdown()) and (num_open_fingers == 2)):

#------------Linear Formation with 2 metre distance between each robot (along y)----------------------            
        #x_target = x_coord_1-x_coord_1+0.5*x_coord2+0.5*x_coord3+0.5*2*2-0.5*1*2-0.5*1*2
        #y_target = y_coord_1-y_coord_1+0.5*y_coord2+0.5*y_coord3+0.5*2*2-0.5*1*4-0.5*1*6 + 0.5
#-----------------------------------------------------------------------------------------------------

#------------Linear Formation with 1 metre distance between each robot (along y)----------------------
        #x_target = x_coord_1-x_coord_1+0.5*x_coord2+0.5*x_coord3+0.5*2*2-0.5*1*2-0.5*1*2
        #y_target = y_coord_1-y_coord_1+0.5*y_coord2+0.5*y_coord3+0.5*2*2-0.5*1*3-0.5*1*4 +0.5 #to make the formation move along y-axis
#-----------------------------------------------------------------------------------------------------

#------------Horizontal Formation with 1 metre distance between each robot (along x)-----------------------
        #x_target = x_coord_1-x_coord_1+0.5*x_coord2+0.5*x_coord3+0.5*2*0.5-0.5*1*1.5-0.5*1*2.5
        #y_target = y_coord_1-y_coord_1+0.5*y_coord2+0.5*y_coord3+0.5*2*1-0.5*1*1-0.5*1*1 + 0.5 #to make the formation move along y-axis

#------------------------------------------------------------------------------------------------

#------------Triangular Formation (Isosceles)----------------------------------------------------
        #x_target = x_coord_1-x_coord_1+0.5*x_coord2+0.5*x_coord3+0.5*2*1-0.5*1*3-0.5*1*2
        #y_target = y_coord_1-y_coord_1+0.5*y_coord2+0.5*y_coord3+0.5*2*1-0.5*1*1-0.5*1*2 + 0.5 #to make the formation move along y-axis
#------------------------------------------------------------------------------------------------
 
 
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&----4 ROBOTS---&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

#-------------Horizontal Formation with 1 metre distance between each robot (along x)-----------
        x_target = x_coord_1 - 0.5*x_coord_1 + 0.5*x_coord3 + 0.5*1*1 - 0.5*1*3
        y_target = y_coord_1 - 0.5*y_coord_1 + 0.5*y_coord3 + 0.5*1*2 - 0.5*1*2 + 0.5
#------------------------------------------------------------------------------------------------

        #rospy.loginfo("target_x = %f , target_y = %f",x_target,y_target)
            
        diff_x = x_target - x_coord_1
        diff_y = y_target - y_coord_1
# Finding the Target Angle
        #rospy.loginfo("difference in x:%f difference in y:%f ",diff_x,diff_y)
        target_ang = math.atan2(diff_y,diff_x)
        #rospy.loginfo("Target Angle = %f",target_ang)
            
#Angle Reading from /odom (Converted to Euler Angles from Quaternion)
        yaw1 = yaw
        #rospy.loginfo("Current Angle = %f", yaw1)
            
        diff_ang1 = target_ang-yaw1
     
            
        if (np.sign(target_ang) == np.sign(yaw1)):
            diff_ang = diff_ang1
                
        elif (np.sign(target_ang) != np.sign(yaw1)) and (abs(diff_ang1) <= math.pi):
            diff_ang = diff_ang1
                
        elif (np.sign(target_ang) != np.sign(yaw1)) and (abs(diff_ang1) > math.pi):
            if (diff_ang1 < 0):
                diff_ang = (2*math.pi)-abs(diff_ang1)
            elif (diff_ang1 > 0):
                diff_ang = (-2*math.pi)+abs(diff_ang1)
                    
        rospy.loginfo("*Adjusted* Angle by which robot will rotate = %f ",diff_ang)
                    
#To avoid discontinuity at cos(pi/2)...since as cosine will begin to decrease near pi/2,  the  linear velocity will increase a lot
        diff_ang2 = diff_ang
        if (1<=diff_ang<=1.57):
            diff_ang2 = 1
        if (1.57<diff_ang<=2.14):  
            diff_ang2 = 2.14
                
        if (-1.57<=diff_ang<=-1):
            diff_ang2 = -1
        if (-2.14<diff_ang<=-1.57): #to avoid discontinuity at cos(-pi/2)
            diff_ang2 = -2.14
        dif_x = x_target-x_coord_1
        dif_y = y_target-y_coord_1
         
   
        lin_vel = kp1* math.sqrt(math.pow(dif_x,2)+math.pow(dif_y,2))/(0.1*abs(math.cos(diff_ang2)))

        ang_vel = kp2*(diff_ang)/0.1

        rospy.loginfo("Stage 3:Linear Velocity = %f , Angular Velocity = %f",lin_vel,ang_vel)
        command.linear.x = lin_vel
        command.angular.z = ang_vel
            
        pub.publish(command)
                
        diff_x = x_target-x_coord_1
        diff_y = x_target-y_coord_1
        x_sq = math.pow(diff_x,2)
        y_sq = math.pow(diff_y,2)
        dist = math.sqrt(x_sq+y_sq)
               
        #toc1 = time.perf_counter()   
        #if ((toc1-tic1) >= 180):
            #break
        r.sleep()
        
      
#Function corresponding to 3 open fingers      
def make_form_stop():

    while ( (not rospy.is_shutdown()) and (num_open_fingers==3) ):
        command.linear.x = 0
        command.angular.z = 0
        pub.publish(command)
        r.sleep()
        
        
        
                

if __name__=='__main__':

    try:
               
        rospy.init_node('gest_cont_form_1')
        rospy.Subscriber('/hedge_pos_ang',hedge_pos_ang,get_pos)
        rospy.Subscriber('robot1/odom',Odometry, get_rotation)
        rospy.Subscriber('robot1/mobile_base/events/bumper',BumperEvent,detect_coll)
        rospy.Subscriber('finger_count', UInt8,callback)
        pub = rospy.Publisher('robot1/mobile_base/commands/velocity', Twist, queue_size=10)
       
        
        #r = rospy.Rate(10)
        r = rospy.Rate(20)
        r1 = rospy.Rate(20)
        #r2 = rospy.Rate(20)
        command = Twist()
        vel = Twist()
        
        time.sleep(2)
        
        while not rospy.is_shutdown():
        
            rospy.loginfo("Number of open fingers is %d ", num_open_fingers)
        
            if (num_open_fingers == 1):
                get_into_form()
            
            elif (num_open_fingers == 2):
                make_form_move()
            
            elif (num_open_fingers == 3):
                make_form_stop()
                
            r.sleep()
            
    
    except rospy.ROSInterruptException:
        pass
