#!/usr/bin/env python

# import rospy
import socket
import threading

# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import LaserScan

# GLOBAL CONSTANTS
HEADER = 512 # réponse de 16 octets, donc
FORMAT = "utf-8"
DISCONNECT_MESSAGE = "!DISCONNECT"
MAX_CONNECTIONS = 10

# GLOBAL REMOTE REQUEST CONSTANTS

REMOTE_REQUEST_PORT = 65432
REMOTE_REQUEST_HOST = socket.gethostbyname(socket.gethostname())
REMOTE_REQUEST_ADDR = (REMOTE_REQUEST_HOST , REMOTE_REQUEST_PORT)

# GLOBAL POS BROADCAST CONSTANTS

POS_BROADCAST_PORT = 65431
POS_BROADCAST_HOST = "??"
POS_BROADCAST_ADDR = (REMOTE_REQUEST_HOST , REMOTE_REQUEST_PORT)

# SERVER CREATION

# socket.AF_INET means we're using IPv4 ( IP version 4 )
# socket.SOCK_STREAM means we're using TCP protocol for data transfer

REMOTE_REQUEST_SERVER = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
REMOTE_REQUEST_SERVER.bind(REMOTE_REQUEST_ADDR)

POS_BROADCAST_SERVER = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
POS_BROADCAST_SERVER.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
POS_BROADCAST_SERVER.bind(POS_BROADCAST_ADDR)


class ROSMonitor:

    def __init__(self):
        # Add your subscriber here (odom? laserscan?):
        # self.sub_odom = rospy.Subcriber(...)
        # self.sub_laser = rospy.Subscriber(...)

        # Current robot state:
        self.id = 0xFFFF
        self.pos = (0,0,0)
        self.obstacle = False

        # Params :
        # self.remote_request_port = rospy.get_param("remote_request_port", REMOTE_REQUEST_PORT)
        # self.pos_broadcast_port  = rospy.get_param("pos_broadcast_port", POS_BROADCAST_PORT)

        print("ROSMonitor started.")

        # Thread for RemoteRequest handling:
        self.rr_thread = threading.Thread(target=self.wait_for_connection).start()


    def wait_for_connection(self):

        REMOTE_REQUEST_SERVER.listen(MAX_CONNECTIONS)

        print(f"[LISTENING] Server is listenning on {REMOTE_REQUEST_HOST}")

        while True:
            try:
                socket, addr = REMOTE_REQUEST_SERVER.accept()
                self.id = addr

                print(f"[CONNECTION] {addr} connected to the server")

                thread = threading.Thread(target=self.handle_client, args=(socket, addr)).start()
                print(f"[ACTIVE CONNECTIONS] {threading.active_count() - 1}") ######### À revérifier

            except Exception as e:
                print("[EXCEPTION]", e)
                break
        
        print("SERVER CRASHED")

    def handle_client(self, socket, addr):

        socket.send("[CONNECTION] Connection to ROSMonitor sucessful:\n\n".encode(FORMAT))
        socket.send("What ASCII command to retreive info:\n1.RPOS\n2.OBSF\n3.RBID\n".encode(FORMAT))

        while True:
            msg_length = socket.recv(HEADER).decode(FORMAT)# Get the size of the message we want to receive
            if msg_length:
                msg_length = int(msg_length)
                msg = socket.recv(msg_length).decode(FORMAT)# Get message of good size( not more or less )
                if msg == DISCONNECT_MESSAGE:

                    print(f"[DISCONNECTION] {addr} disconnected from the server")
                    socket.close()
                    print(f"[ACTIVE CONNECTIONS] {threading.active_count() - 2}") ######### À revérifier
                    break

                print(f"[{addr}] {msg}")
                socket.send("What ASCII command to retreive info:\n1.RPOS\n2.OBSF\n3.RBID\n".encode(FORMAT))
                


if __name__=="__main__":
    # rospy.init_node("ros_monitor")

    node = ROSMonitor()

    # rospy.spin()


