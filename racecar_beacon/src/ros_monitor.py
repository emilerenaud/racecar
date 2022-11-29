#!/usr/bin/env python3

import rospy
import socket
import struct
import threading

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion


def quaternion_to_yaw(quat):
    return euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[2]


class ROSMonitor:
    def __init__(self):
        # Subscribers :
        self.sub_odom = rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        self.sub_laser = rospy.Subscriber("/scan", LaserScan, self.scan_cb)
        # self.sub_odom = rospy.Subscriber("/racecar/odometry/filtered",Odometry,
                                         self.update_position)
        # self.sub_laser = rospy.Subscriber("/racecar/scan", LaserScan, 
                                          self.detect_obstacle)

        # Current robot state :
        self.id = 12345
        self.pos = (0, 0, 0)
        self.obstacle = False

        # Params :
        self.remote_request_port = rospy.get_param("remote_request_port", 65432)
        self.remote_request_host = rospy.get_param("remote_request_host", "10.0.1.31")
        self.pos_broadcast_port = rospy.get_param("pos_broadcast_port", 65431)
        self.pos_broadcast_host = rospy.get_param("pos_broadcast_host", "10.0.1.255")

        # Sockets :
        self.pos_broadcast_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.pos_broadcast_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.pos_broadcast_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # Thread for RemoteRequest handling :
        self.rr_thread = threading.Thread(target=self.rr_loop)
        self.rr_thread.daemon = True # To be able to use Ctrl-C to stop the program
        self.rr_thread.start()

        # Creates a ROS Timer that will call PositionBroadcast every 1.0 sec:
        self.timer = rospy.Timer(rospy.Duration(1.0), self.position_broadcast)

        print("ROSMonitor started.")

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = quaternion_to_yaw(msg.pose.pose.orientation)
        self.pos = (x, y, theta)

    def scan_cb(self, msg):
        self.obstacle = min(msg.ranges) < 1

    def rr_loop(self):
        self.rr_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.rr_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.rr_socket.bind((self.remote_request_host, self.remote_request_port))

        while True:
            try:
                # Start the connection
                self.rr_socket.listen(1)
                (conn, addr) = self.rr_socket.accept()

                stop = False
                while not stop:
                    # Get the request code
                    cmd = conn.recv(1024).decode()

                    # Get the info
                    if cmd == "RPOS":
                        msg = self.get_position()
                    elif cmd == "OBSF":
                        msg = self.get_obstacle()
                    elif cmd == "RBID":
                        msg = self.get_id()
                    elif cmd == "exit":
                        msg = self.get_invalid_msg()
                        stop = True
                    else:
                        msg = self.get_invalid_msg()

                    # Send the message
                    conn.send(msg)

                # Close the connection
                conn.close()
            except socket.timeout:
                pass

    def get_position(self):
        return struct.pack("!3f4x", self.pos[0], self.pos[1], self.pos[2])

    def get_obstacle(self):
        return struct.pack("!I12x", 1 if self.obstacle else 0)

    def get_id(self):
        return struct.pack("!I12x", self.id)

    def get_invalid_msg(self):
        return struct.pack("!16x")

    def position_broadcast(self, event):
        format = "!3fI"
        data = struct.pack(format, self.pos[0], self.pos[1], self.pos[2], self.id)
        self.pos_broadcast_socket.sendto(data, (self.pos_broadcast_host, self.pos_broadcast_port))


    def manage_requests(self, client_sock):
        rqst_format = "cccc"

        while True: 
            client_rqst = client_sock.recv(4)
            
            if not client_rqst:
                return
        
            try:               
                rqst_bytes = struct.unpack(rqst_format, client_rqst)

                rqst_str = (rqst_bytes[0].decode('ascii') + 
                            rqst_bytes[1].decode('ascii') + 
                            rqst_bytes[2].decode('ascii') + 
                            rqst_bytes[3].decode('ascii'))

                if rqst_str == "RPOS":
                    resp_str = self.position_rqst()
                elif rqst_str == "OBSF":
                    resp_str = self.obstacle_rqst()
                elif rqst_str == "RBID":
                    resp_str = self.id_rqst()
                else:
                    raise ValueError

            except struct.error:
                raise ValueError
            except ValueError:
                resp_format = "ixxxxxxxxxxxx"
                resp_str = struct.pack(resp_format, -1)

            client_sock.send(resp_str)

    def position_rqst(self):
        resp_format = "fffxxxx"
        return struct.pack(resp_format, self.pos[0], self.pos[1], self.pos[2])

    def obstacle_rqst(self):
        resp_format = "Ixxxxxxxxxxxx"
        return struct.pack(resp_format, self.obstacle)

    def id_rqst(self):
        resp_format = "Ixxxxxxxxxxxx"
        return struct.pack(resp_format, self.id)


    def update_position(self, msg):
        position = msg.pose.pose.position
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.pos = (position.x, position.y, yaw)

    def detect_obstacle(self, msg):
        dist_treshold = 1.00

        for dist in msg.ranges:
            if dist == "inf":
                pass
            elif dist < dist_treshold:
                self.obstacle = 1
                return

        self.obstacle = 0

    def broadcast_position(self, event):
        pos_msg_format = "fffI"
        pos_msg = struct.pack(pos_msg_format, 
                       self.pos[0], 
                       self.pos[1], 
                       self.pos[2], 
                       self.id)
        
        self.bc_socket.sendto(pos_msg, (self.pos_broadcast_ip, self.pos_broadcast_port))

        #try:
        #    self.bc_socket.sendto(pos_msg, (self.pos_broadcast_ip, self.pos_broadcast_port))
        #except Exception as e:
        #    self.bc_socket.sendto(pos_msg, (self.pos_broadcast_ip, self.pos_broadcast_port))


# if __name__== "__main__" :
#     rospy.init_node("ros_monitor")
# 
#     node = ROSMonitor()
# 
#     node.rr_thread.setDaemon(1)
#     node.rr_thread.start()
#     
#     rospy.spin()
    
if __name__ == "__main__":
    try:
        rospy.init_node("ros_monitor")

        node = ROSMonitor()

        rospy.spin()
    except KeyboardInterrupt:
        node.pos_broadcast_socket.close()
        node.rr_socket.close()
