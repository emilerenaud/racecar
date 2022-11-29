#!/usr/bin/env python3

import socket
import struct

HOST = '10.0.1.255'
# This process should listen to a different port than the RemoteRequest client.
PORT = 65431
format = "!3fI"

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind((HOST, PORT))

try:
    print("Press Enter to receive a message.\nType 'exit' to close the program.\n")
    stop = False
    while not stop:
        msg = input("")

        if msg == "exit":
            stop = True
        else:
            (x, y, theta, id)  = struct.unpack(format, s.recvfrom(1024)[0])
            print(f"{HOST}:{PORT}\n")
            print(f"   x     : {x}\n")
            print(f"   y     : {y}\n")
            print(f"   theta : {theta}\n")
            print(f"   id    : {id}\n")

except KeyboardInterrupt:
    pass

s.close()
