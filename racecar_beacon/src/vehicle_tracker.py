#!/usr/bin/env python

import socket
import time

from struct import *

HEADER = 128  # 16 octets x 8 = 128 bit message
PORT = 65431
FORMAT = 'utf-8'
ADDR = ('', PORT)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock.bind(ADDR)

def connection():

    while True:
        try:
            received_msg, addr = sock.recvfrom(HEADER)

            received_msg = unpack(">fffi", received_msg)

            received_msg = list(received_msg)

            received_msg[3] = pack(">i",received_msg[3])

            received_msg[3] = socket.inet_ntoa(received_msg[3])

            received_msg = tuple(received_msg)

            print(received_msg)
        
        except:
            print('[DISCONNECT] Vehicle Tracker has been disconnected')
            break


if __name__ == "__main__":

    connection()


