#!/usr/bin/env python3

import socket
import struct

def get_position(data):
    return struct.unpack("!3f4x", data)

def get_obstacle(data):
    return struct.unpack("!I12x", data)[0] == 1

def get_id(data):
    return struct.unpack("!I12x", data)[0]

def get_invalid_msg(data):
    return struct.unpack("!16x", data)


def print_position(pos):
    print("Position\n")
    print(f"  X     : {pos[0]}\n")
    print(f"  Y     : {pos[1]}\n")
    print(f"  Theta : {pos[2]}\n")

def print_obstacle(obs):
    print(f"Obstacle : {obs}\n")

def print_id(id):
    print(f"ID : {id}\n")


HOST = '10.0.1.31'
# This process should listen to a different port than the PositionBroadcast client.
PORT = 65432

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

print("Type your request. (RPOS, OBSF, RBID, exit)\n")

try:
    stop = False
    while not stop:
        # Read from keyboard
        cmd = input("> ")

        if cmd == "":
            print("Invalid command.\n")
        else:
            s.send(cmd.encode())
            data = s.recv(1024)

            # Parse data
            if cmd == "RPOS":
                print_position(get_position(data))
            elif cmd == "OBSF":
                print_obstacle(get_obstacle(data))
            elif cmd == "RBID":
                print_id(get_id(data))
            elif cmd == "exit":
                stop = True
            else:
                print(f"Invalid command.\n")

except KeyboardInterrupt:
    pass

s.close()