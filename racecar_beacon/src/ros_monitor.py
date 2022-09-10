#!/usr/bin/env python

from lab_poll_pos import quaternion_to_yaw
import rospy
import socket
import threading

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from struct import *


class ROSMonitor:
    def __init__(self):
        # Add your subscriber here (odom? laserscan?):

        self.sub_laser = rospy.Subscriber("/scan", LaserScan, self.scan_laser)
        self.sub_odom = rospy.Subscriber("/odometry/filtered", Odometry, self.scan_odom)
        # Current robot state:
        self.id = pack("Ixxx",0xFFFF)
        self.pos = 0
        self.obstacle = pack("Ixxx",0)

        # Params :
        self.remote_request_port = rospy.get_param("remote_request_port", 65432)
        self.pos_broadcast_port  = rospy.get_param("pos_broadcast_port", 65431)

        # Thread for RemoteRequest handling:
        self.rr_thread = threading.Thread(target=self.rr_loop)

        print("ROSMonitor started.")

    def rr_loop(self):
        # Init your socket here :
        # self.rr_socket = socket.Socket(...)
        while True:
            pass

    def quaternion_to_yaw(quat):
    # Uses TF transforms to convert a quaternion to a rotation angle around Z.
    # Usage with an Odometry message: 
    #   yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        (roll, pitch, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        return yaw

    def scan_laser(self,msg):
        # print(msg.ranges)
        rangesValues = msg.ranges
        for data in rangesValues:
            if data < 1:
                print("Data",data)
                self.obstacle = pack("Ixxx",1)

        
        # pass

    def scan_odom(self,msg):
        print('X: ',msg.pose.pose.position.x)
        print('Y: ',msg.pose.pose.position.y)
        print('Theta: ',quaternion_to_yaw(msg.pose.pose.orientation))
        self.pos = pack("fffx",msg.pose.pose.position.x,msg.pose.pose.position.y,quaternion_to_yaw(msg.pose.pose.orientation))
        


if __name__=="__main__":
    rospy.init_node("ros_monitor")

    node = ROSMonitor()

    rospy.spin()


