#!/usr/bin/env python

import socket
import threading
import time

HOST = '127.0.0.1'

# This process should listen to a different port than the RemoteRequest client.
PORT = 65431

if __name__=="__main__":
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) # UDP
    client.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)

# Enable broadcasting mode
    client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    client.bind(("", PORT))
    while True:
        # Thanks @seym45 for a fix
        data, addr = client.recvfrom(1024)
        print("received message: %s"%data)

