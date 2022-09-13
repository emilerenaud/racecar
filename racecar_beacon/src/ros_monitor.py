#!/usr/bin/env python

import rospy
import roslaunch
import socket
import threading
import time

from struct import *
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

# GLOBAL CONSTANTS
HEADER = 128  # réponse de 16 octets, donc 128 bits
FORMAT = "utf-8"
DISCONNECT_MESSAGE = "DISCONNECT"
MAX_CONNECTIONS = 10


# GLOBAL REMOTE REQUEST CONSTANTS

REMOTE_REQUEST_PORT = 65432
# REMOTE_REQUEST_HOST = socket.gethostbyname(socket.gethostname())
REMOTE_REQUEST_HOST = socket.gethostbyname(socket.gethostname()+".local")
REMOTE_REQUEST_ADDR = (REMOTE_REQUEST_HOST, REMOTE_REQUEST_PORT)

# GLOBAL POS BROADCAST CONSTANTS


POS_BROADCAST_PORT = 65431
POS_BROADCAST_HOST = ""

# SERVER CREATION

# socket.AF_INET means we're using IPv4 ( IP version 4 )
# socket.SOCK_STREAM means we're using TCP protocol for data transfer

REMOTE_REQUEST_SERVER = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
REMOTE_REQUEST_SERVER.bind(REMOTE_REQUEST_ADDR)

POS_BROADCAST_SERVER = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
POS_BROADCAST_SERVER.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

def quaternion_to_yaw(quat):
    # Uses TF transforms to convert a quaternion to a rotation angle around Z.
    # Usage with an Odometry message: 
    #   yaw = quaternion_to_yaw(msg.pose.pose.orientation)
    (roll, pitch, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    return yaw

class ROSMonitor:

    def __init__(self):
        # Add your subscriber here (odom? laserscan?):
        self.sub_odom = rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        # self.sub_laser = rospy.Subscriber(...)

        # Current robot state:
        self.id = 0xFFFF
        self.pos = (0, 0, 0)
        self.obstacle = False
        self.rate = rospy.Rate(1)


        # Params :
        # self.remote_request_port = rospy.get_param("remote_request_port", REMOTE_REQUEST_PORT)
        # self.pos_broadcast_port  = rospy.get_param("pos_broadcast_port", POS_BROADCAST_PORT)

        print("ROSMonitor started.")

        self.get_site_number(REMOTE_REQUEST_HOST)

        # Thread for RemoteRequest handling:
        self.rr_thread = threading.Thread(
            target=self.wait_for_connection, daemon = True).start()

        # Thread for VehicleTracking handling:
        self.vt_thread = threading.Thread(
            target=self.broadcast, args = (self.rate,), daemon = True).start()

    def get_site_number(self, address_string):
        global POS_BROADCAST_HOST
        dot_counter = 0
        site_number = ""
        for character in address_string:
            if dot_counter == 2 and character != ".":
                site_number = site_number + character
            if character == ".":
                dot_counter = dot_counter + 1
        POS_BROADCAST_HOST = "10.0."+ site_number+".255"

    """
    .########..########...#######.....###....########...######.....###.....######..########
    .##.....##.##.....##.##.....##...##.##...##.....##.##....##...##.##...##....##....##...
    .##.....##.##.....##.##.....##..##...##..##.....##.##........##...##..##..........##...
    .########..########..##.....##.##.....##.##.....##.##.......##.....##..######.....##...
    .##.....##.##...##...##.....##.#########.##.....##.##.......#########.......##....##...
    .##.....##.##....##..##.....##.##.....##.##.....##.##....##.##.....##.##....##....##...
    .########..##.....##..#######..##.....##.########...######..##.....##..######.....##...
    """
    
    def odom_cb(self, msg):
        """Feeds ros_monitor node odometry position to self.pos

        Args:
            msg (Odometry): Gets message as subscriber of "/odometry/filtered" topic
        """

        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.pos = (msg.pose.pose.position.x,msg.pose.pose.position.y,yaw)

    def broadcast(self, rate):
        """Broadcast vehicle position and server IP 

        Args:
            rate (rospy.Rate): Send a message every 1Hz
        """        

        print(f"[BROADCAST] Server is broadcasting on {POS_BROADCAST_HOST}")

        while True:

            msg = pack(">fffi",self.pos[0],self.pos[1],self.pos[2],self.id)
            POS_BROADCAST_SERVER.sendto(msg, (POS_BROADCAST_HOST, POS_BROADCAST_PORT ))
            rate.sleep()

    """
    ..######..##.......####.########.##....##.########
    .##....##.##........##..##.......###...##....##...
    .##.......##........##..##.......####..##....##...
    .##.......##........##..######...##.##.##....##...
    .##.......##........##..##.......##..####....##...
    .##....##.##........##..##.......##...###....##...
    ..######..########.####.########.##....##....##...
    """

    def wait_for_connection(self):

        """
        Waits for client to connect to server, then starts a thread to handle client messages when connected
        """

        self.id = unpack("!i", socket.inet_aton(REMOTE_REQUEST_HOST))[0]
        REMOTE_REQUEST_SERVER.listen(MAX_CONNECTIONS)

        print(f"[LISTENING] Server is listenning for client on {REMOTE_REQUEST_HOST}")

        while True:
            try:
                conn, addr = REMOTE_REQUEST_SERVER.accept()

                print(f"[CONNECTION] {addr} connected to the server")

                thread = threading.Thread(
                    target=self.handle_client, args=(conn, addr), daemon= True).start()
    
                # print(f"[ACTIVE CONNECTIONS] {threading.active_count() - 1}")# À revérifier

            except Exception as e:
                print("[EXCEPTION]", e)
                break

        print("SERVER CRASHED")
        socket.close()

    def handle_client(self, socket, addr):
        """Handles the mesage received by the client

        Args:
            socket (socket): Has client info
            addr (tuple(IP,port)): Has IP and port number to client connection
        """

        socket.send(
            "[CONNECTION] Connection to ROSMonitor sucessful:\n\nWhat ASCII command to retreive info:\n1.RPOS\n2.OBSF\n3.RBID\n".encode(FORMAT))

        while True:

            if not socket.timeout:
                msg = socket.recv(HEADER).decode(FORMAT)
            else:
                print(f"[TIMEOUT] {addr} has timed out of the server")
                break

            if msg and msg != DISCONNECT_MESSAGE:
                print(f"[{addr}] {msg}")
                self.send_client(socket, msg)

            elif msg == DISCONNECT_MESSAGE:

                print(
                    f"[DISCONNECTION] {addr} disconnected from the server")
                socket.close()
                # print(
                #     f"[ACTIVE CONNECTIONS] {threading.active_count() - 3}")# À revérifier
                break
            
    
    def send_client(self,socket, msg):
        """Send message back to client

        Args:
            socket (socket): Has client info
            msg (string): Message that the client sent to server to get info
        """

        if msg == "RPOS":
            msg = pack(">fffx",self.pos[0],self.pos[1],self.pos[2])

        elif msg == "OBSF":

            msg = pack(">ixxx", int(self.obstacle)) # BON nbre de x?

        elif msg == "RBID":
            msg = pack(">ixxx",self.id)

        socket.send(msg)
        time.sleep(0.1)
        socket.send(
            "What ASCII command to retreive info:\n1.RPOS\n2.OBSF\n3.RBID\n".encode(FORMAT))



if __name__ == "__main__":

    # uuid = roslaunch.rlutil.get_or_generate_uuid(options_runid=None, options_wait_for_master=False)
    # roslaunch.configure_logging(uuid)
    # launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_files=[], is_core=True)
    # launch.start()

    rospy.init_node("ros_monitor")

    node = ROSMonitor()

    rospy.spin()
    REMOTE_REQUEST_SERVER.close()
    # launch.shutdown()
