#!/usr/bin/env python

import rospy
import math 
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray

class PathFollowing:
    def __init__(self):
        self.max_speed = rospy.get_param('~max_speed', 1)
        self.max_steering = rospy.get_param('~max_steering', 0.37)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size=1)

        # Camera shit
        self.balloon_pub = rospy.Subscriber('position_balloon',Float32MultiArray, self.balloon_callback, queue_size=10)
        self.balloon_list = list()
        # self.balloon_list.append((0,0))
        self.bridge = CvBridge()    
        self.balloon_path = r'/home/emile/racecar_balloon/'
        print(os.getcwd())


        time.sleep(1)    

    def scan_callback(self, msg):
        # Because the lidar is oriented backward on the racecar, 
        # if we want the middle value of the ranges to be forward:
        #l2 = len(msg.ranges)/2;
        #ranges = msg.ranges[l2:len(msg.ranges)] + msg.ranges[0:l2]
        
        twist = Twist()
        twist.linear.x = self.max_speed
        twist.angular.z = 0
           
        self.cmd_vel_pub.publish(twist)
        
    def odom_callback(self, msg):
        # rospy.loginfo("Current speed = %f m/s", msg.twist.twist.linear.x)
        pass

    def balloon_callback(self,msg):
        # rospy.loginfo("Balloon position = %f, %f", msg.data[0],msg.data[1])
        # Position d'une balloon
        pos_x = msg.data[0]
        pos_y = msg.data[1]
        distance = msg.data[2]
        angle = msg.data[3]
        tolerance = 1
        new_balloon = 1

        # Verifier si on a deja enregristrer cette position
        if(len(self.balloon_list) == 0):
            self.balloon_list.append((pos_x,pos_y))
            rospy.loginfo("FIRST BALloON")
            self.take_picture()
        else:
            
            for balloon in self.balloon_list:
                if (math.hypot((abs(pos_x - balloon[0])),(abs(pos_y - balloon[1]))) <= tolerance):
                    new_balloon = 0
                    break
            if new_balloon == 1:
                rospy.loginfo("New ball")
                self.balloon_list.append((pos_x,pos_y))
                self.take_picture()
            

            # test = filter(math.hypot((abs(pos_x - balloon[0])),(abs(pos_y - balloon[1]))) > tolerance,self.balloon_list)
            # print(test)
            # for balloon in self.balloon_list:
            #     # rospy.loginfo("FOR")
            #     # rospy.loginfo("X : %f + Y : %f",abs(pos_x - balloon[0]),abs(pos_y - balloon[1]))
            #     if (math.hypot((abs(pos_x - balloon[0])),(abs(pos_y - balloon[1]))) > tolerance):
            #         new_balloon = 1
            #         # rospy.loginfo("New ball")
            #         # # self.balloon_list.append(zip(msg.data[0],msg.data[1]))
            #         # self.balloon_list.append((pos_x,pos_y))
            #         # self.take_picture(pos_x,pos_y)
            #         # rospy.loginfo("data Zip : %f",zip(msg.data[0],msg.data[1]))
            #         break
            

            
            # Se rendre au debris si nouveau debris


    def take_picture(self):
        # print("Received an image!")
        # print(os.listdir())
        try:
            # Convert your ROS Image message to OpenCV2
            rospy.loginfo("Wait camera")
            img_msg = rospy.wait_for_message('/racecar/raspicam_node/image', Image, timeout=15)
            # rospy.loginfo("Wait done camera")
            cv2_img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError:
            print("error")
            rospy.loginfo("Error")
        else:
            # Save your OpenCV2 image as a jpeg 
            os.chdir(self.balloon_path)
            filename = "photo_balloon" + str(len(self.balloon_list)) + ".png"
            cv2.imwrite(filename, cv2_img)
            # cv2.imwrite('camera_image.jpeg', cv2_img)
        rospy.loginfo("File saved")

        

def main():
    rospy.init_node('path_following')
    pathFollowing = PathFollowing()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

