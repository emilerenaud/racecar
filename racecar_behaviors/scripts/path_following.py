#!/usr/bin/env python

import time
import numpy as np
import rospy
import actionlib
import os

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from libbehaviors import *
from path_planning import get_shortest_path_image


from cv_bridge import CvBridge, CvBridgeError
import cv2


class PathFollowing:
    def __init__(self):
        self.max_speed = rospy.get_param('~max_speed', 1)
        self.max_steering = rospy.get_param('~max_steering', 0.37)
        self.current_angle = 0

        # Subscribers
        self.blob_detector_sub = rospy.Subscriber("object_detected", Pose, self.blob_detected_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size=1)

        # Goals
        self.goal_start = self.create_goal(0, 0, np.pi, "pos")
        self.goal_end = self.create_goal(13, 0.5, 0, "pos")
        self.goal_end_r = self.create_goal(13, 0.5, np.pi, "pos")
        self.goals = [self.goal_end, self.goal_end_r, self.goal_start]

        # Action clients
        self.mb_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # Debris
        self.debris_list = []

        # Picture
        self.bridge = CvBridge()
        self.objects_detected = 0
        self.folder_path = "/home/racecar/debris/"
        self.img_paths = []

        time.sleep(1)
        self.mb_client.wait_for_server()
        self.send_goal()

    def create_goal(self, x, y, angle, type):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "racecar/map"
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        (quat_x, quat_y, quat_z, quat_w) = yaw_to_quaternion(angle)
        goal.target_pose.pose.orientation.x = quat_x
        goal.target_pose.pose.orientation.y = quat_y
        goal.target_pose.pose.orientation.z = quat_z
        goal.target_pose.pose.orientation.w = quat_w
        return (goal, type)

    def send_goal(self):
        if len(self.goals) > 0:
            self.mb_client.send_goal(self.goals[0][0], self.goal_reached_callback)

    def goal_reached_callback(self, goal_status, result):
        goal_x = self.goals[0][0].target_pose.pose.position.x
        goal_y = self.goals[0][0].target_pose.pose.position.y
        goal_angle = quaternion_to_yaw(self.goals[0][0].target_pose.pose.orientation)
        rospy.loginfo("Goal reached!\nX : {0}\nY : {1}\nAngle : {2}\n".format(goal_x, goal_y, goal_angle))

        if (self.goals[0][1] == "blob"):
            self.take_picture()
        self.goals.pop(0)

        if (len(self.goals) == 0):
            self.export_report()
        else:
            self.send_goal()

    def blob_detected_callback(self, msg):
        # Extract blob position from message
        blob_x = msg.position.x
        blob_y = msg.position.y
        tolerance = 1
        new_debris = True

        # Check if we've seen the debris before
        for debris in self.debris_list:
            rayon = np.sqrt((blob_x - debris[0])**2 + (blob_y - debris[1])**2)
            new_debris = rayon > tolerance
            if not new_debris:
                break

        if new_debris:
            self.debris_list.append((blob_x, blob_y))
            angle = msg.orientation.z
            distance = 1.5 # From the blob to take the picture
            goal_x = blob_x - distance * np.cos(self.current_angle + angle)
            goal_y = blob_y - distance * np.sin(self.current_angle + angle)
            goal = self.create_goal(goal_x, goal_y, self.current_angle + angle, "blob")

            self.goals.insert(0, goal)
            self.send_goal()

    def odom_callback(self, msg):
        self.current_angle = quaternion_to_yaw(msg.pose.pose.orientation)

    def take_picture(self):
        rospy.loginfo("Taking picture...")
        img_msg = rospy.wait_for_message('/racecar/raspicam_node/image', Image)
        rospy.loginfo("Got message from /racecar/raspicam_node/image...")
        cv2_img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

        rospy.loginfo("Got image, creating filename...")
        self.img_filename = "photo_debris_" + str(self.objects_detected + 1) + ".png"
        self.img_paths.append(self.folder_path + self.img_filename)

        rospy.loginfo("Writing image to path...")
        cv2.imwrite(self.img_paths[self.objects_detected], cv2_img)
        self.objects_detected += 1
        rospy.loginfo('Picture taken!\n')
        time.sleep(5)

    def export_report(self):
        report_path = self.folder_path + "report.txt"
        trajectory_paths = []
        with open(report_path, "w") as file:
            i = 0
            for debris in self.debris_list:
                trajectory_paths.append(self.folder_path + "trajectory_debris_{0}.png".format(i + 1))
                file.write("Debris {0} :\n   X : {1:.2f}\n   Y : {2:.2f}\n   Picture path : {3}\n   Trajectory path : {4}\n\n".format(i + 1, debris[0], debris[1], self.img_paths[i], trajectory_paths[i]))
                i += 1
            rospy.loginfo("Report exported at {0}".format(report_path))

        # Generate the trajectory to reach each debris
        # We do this in a separate loop because it's quite long, we want the
        # text report to be generated quickly
        for i in range(0, len(self.debris_list)):
            get_shortest_path_image("a_star", trajectory_paths[0], 0, 0, self.debris_list[i][0], self.debris_list[i][1])


def main():
    rospy.init_node('path_following')
    pathFollowing = PathFollowing()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

