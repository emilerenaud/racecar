#!/usr/bin/env python

import rospy
import math 
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
import time
import actionlib
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from libbehaviors import *
from tf.transformations import quaternion_from_euler

class PathFollowing:
    def __init__(self):
        self.max_speed = rospy.get_param('~max_speed', 1)
        self.max_steering = rospy.get_param('~max_steering', 0.37)

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.current_angle = 0

        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size=1)
        self.balloon_sub = rospy.Subscriber('position_balloon',Float32MultiArray, self.balloon_callback, queue_size=10)

        # Camera shit
        self.balloon_list = list()
        self.bridge = CvBridge()    
        self.balloon_path = r'/home/jackson/racecar/debris'

        # Goals
        self.goal_start = self.create_goal(0, 0, np.pi, "pos") # WHY np.pi?
        self.goal_end = self.create_goal(13.5, 2.1, 0, "pos")
        self.goal_end_r = self.create_goal(13.5, 2.1, np.pi, "pos")
        self.goals = [self.goal_end, self.goal_end_r, self.goal_start]

        # Action clients
        self.mb_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        time.sleep(1)
        self.mb_client.wait_for_server()
        self.send_goal()

    def send_goal(self):
        if len(self.goals) > 0:
            self.mb_client.send_goal(self.goals[0][0], self.goal_reached_callback)

    def goal_reached_callback(self):
        goal_x = self.goals[0][0].target_pose.pose.position.x
        goal_y = self.goals[0][0].target_pose.pose.position.y
        goal_angle = quaternion_to_yaw(self.goals[0][0].target_pose.pose.orientation)
        # rospy.loginfo("Goal reached!\nX : {0}\nY : {1}\nAngle : {2}\n".format(goal_x, goal_y, goal_angle))

        if (self.goals[0][1] == "balloon"):

            self.create_goal(goal_x, goal_y, goal_angle + self.balloon_sub.data[3], "picture")

        elif( self.goals[0][1] == "picture" ):
            self.take_picture()
            
        self.goals.pop(0)
        self.send_goal()

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
        self.current_angle = quaternion_to_yaw(msg.pose.pose.orientation)
        pass

    def balloon_callback(self,msg):
        # rospy.loginfo("Balloon position = %f, %f", msg.data[0],msg.data[1])
        # Position d'une balloon
        pos_x = msg.data[0]
        pos_y = msg.data[1]
        distance = msg.data[2]
        angle = msg.data[3]
        tolerance = 1
        new_balloon = True

        # Verifier si on a deja enregristrer cette position
        if(len(self.balloon_list) == 0):
            self.balloon_list.append((pos_x,pos_y))
            rospy.loginfo("FIRST BALloON")
        else:
            
            for balloon in self.balloon_list:
                if (math.hypot((abs(pos_x - balloon[0])),(abs(pos_y - balloon[1]))) <= tolerance):
                    new_balloon = 0
                    break

        if new_balloon:
            rospy.loginfo("New ball")
            self.balloon_list.append((pos_x,pos_y))

            angle = msg.orientation.z
            distance = 1.5 # From the blob to take the picture
            goal_x = pos_x - distance * np.cos(self.current_angle + angle)
            goal_y = pos_y - distance * np.sin(self.current_angle + angle)
            goal = self.create_goal(goal_x, goal_y, self.current_angle + angle, "balloon")

            self.goals.insert(0, goal)
            self.send_goal()


            self.take_picture()
           
            
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
            # Save your OpenCV2 image as a png
            os.chdir(self.balloon_path)
            filename = "photo_balloon" + str(len(self.balloon_list)) + ".png"
            cv2.imwrite(filename, cv2_img)
        rospy.loginfo("File saved")

    def create_goal(self, x, y, angle, type):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "racecar/map"
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        (quat_x, quat_y, quat_z, quat_w) = quaternion_from_euler(0,0,angle)
        goal.target_pose.pose.orientation.x = quat_x
        goal.target_pose.pose.orientation.y = quat_y
        goal.target_pose.pose.orientation.z = quat_z
        goal.target_pose.pose.orientation.w = quat_w
        return (goal, type)
        

def main():
    rospy.init_node('path_following')
    pathFollowing = PathFollowing()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

