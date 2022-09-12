#!/usr/bin/env python

import socket
import time

from struct import *

HEADER = 128  # 16 octets x 8 = 128 bit message
PORT = 65432
FORMAT = 'utf-8'
DISCONNECT_MESSAGE = "DISCONNECT"
HOST = 0xFFFF
ADDR = (0, 0)

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)


def send():

    print(client.recv(2048).decode(FORMAT))  # Print menu for client

    msg = None

    while msg != DISCONNECT_MESSAGE:

        try:

            msg = input()

            if (msg == "1" or msg == "2" or msg == "3" or msg == "RPOS" or msg == "OBSF" or msg == "RBID" or msg == DISCONNECT_MESSAGE):

                if (msg == "1"):
                    msg = "RPOS"

                elif (msg == "2"):
                    msg = "OBSF"

                elif (msg == "3"):
                    msg = "RBID"

                message = msg.encode(FORMAT)
                client.send(message)
                receive_msg(msg)

            else:
                print("[ERROR] Can't send this message. Try again")
        except:
            print("[DISCONNECT] Connection to server has been lost")
            break

def receive_msg(msg):
    try:
        if msg != DISCONNECT_MESSAGE:

            received_msg = client.recv(HEADER)

            if (msg == "RPOS"):
                
                received_msg = unpack(">fffx", received_msg)

            elif (msg == "OBSF"):
                
                received_msg = unpack(">ixxx", received_msg)[0]

            elif (msg == "RBID"):

                addr = unpack(">ixxx", received_msg)[0]
                addr = pack(">i",addr)
        
                received_msg = socket.inet_ntoa(addr)

        print(received_msg)
        print(client.recv(2048).decode(FORMAT)) # Print menu for client
    except:
        print('[DISCONNECT] Disconnected from the server')

def connection():

    print("What IP adress would you like to connect to ?")

    while True:

        HOST = input()

        if HOST != 0xFFFF:
            ADDR = (HOST, PORT)
            break

    client.connect(ADDR)
    send()


if __name__ == "__main__":

    connection()
