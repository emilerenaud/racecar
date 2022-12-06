#!/usr/bin/env python

import rospy
import math 
import actionlib
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from libbehaviors import *
from path_planning import path_to_debris

class PathFollowing:
    def __init__(self):
        self.max_speed = rospy.get_param('~max_speed', 1)
        self.max_steering = rospy.get_param('~max_steering', 0.37)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size=1)

        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.goal_init =self.set_goal(0, 0, np.pi)
        self.goal_end = self.set_goal(13.5, 2.1, 0)
        self.goal_return = self.set_goal(13.5, 2.1, np.pi)
        self.goals = [self.goal_end, self.goal_return, self.goal_init]
        self.client.wait_for_server()
        self.sending_goal()

        self.balloon_list = list()
        self.balloon_path = '/home/racecar/debris/'

    def set_goal(self, x, y, angle):
        (quat_x, quat_y, quat_z, quat_w) = yaw_to_quaternion(angle)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "racecar/map"
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.x = quat_x
        goal.target_pose.pose.orientation.y = quat_y
        goal.target_pose.pose.orientation.z = quat_z
        goal.target_pose.pose.orientation.w = quat_w
        return(goal)

    def sending_goal(self):
        self.client.send_goal(self.goals[0], self.goal_done_callback)

    def goal_done_callback(self, goal_status, result):
        self.goals.pop(0)
        self.sending_goal()

        
    def odom_callback(self, msg):
        rospy.loginfo("Current speed = %f m/s", msg.twist.twist.linear.x)

    
    def export_report(self):

        report_path = self.balloon_path + "report.txt"
        trajectory_paths = []
        with open(report_path, "w") as file:
            i = 0
            for debris in self.balloon_list:
                trajectory_paths.append(self.balloon_path + "trajectory_debris_{0}.png".format(i + 1))
                file.write("Debris {0} :\n   X : {1:.2f}\n   Y : {2:.2f}\n   Picture path : {3}\n   Trajectory path : {4}\n\n".format(i + 1, debris[0], debris[1], self.img_paths[i], trajectory_paths[i]))
                i += 1
            rospy.loginfo("Report exported at {0}".format(report_path))

        # Done in another loop to improve the time to generate the report
        for i in range(0, len(self.balloon_list)):
            path_to_debris(self.goal_init, self.balloon_list[i], trajectory_paths[i])



def main():
    rospy.init_node('path_following')
    pathFollowing = PathFollowing()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

