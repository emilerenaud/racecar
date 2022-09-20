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


def connection():

    print("What IP address would you like to connect to ?")

    while True:
        HOST = input()
    
        ADDR = (HOST, PORT)

        try:
            client.connect(ADDR)
            received_msg = client.recv(HEADER)
            print(received_msg.decode(FORMAT))
            send_to_server()
            break
        except:
            print("[ERROR] Invalid IP address Try again")
            break
    client.close()

def send_to_server():

    print("\nWhat ASCII command to retreive info:\n1.RPOS\n2.OBSF\n3.RBID\n") # Print menu for client

    msg = None

    while msg != DISCONNECT_MESSAGE:

        try:

            msg = input()

            # if (msg == "1" or msg == "2" or msg == "3" or msg == "RPOS" or msg == "OBSF" or msg == "RBID" or msg == DISCONNECT_MESSAGE):

            if (msg == "1"):
                msg = "RPOS"

            elif (msg == "2"):
                msg = "OBSF"

            elif (msg == "3"):
                msg = "RBID"
        

            message = msg.encode(FORMAT)
            client.send(message)
            receive_msg(msg)

        except:
            print("[DISCONNECT] Connection to server has been lost")
            break

def receive_msg(msg):
    try:
        if msg != DISCONNECT_MESSAGE:
            received_msg = client.recv(HEADER)
            if (msg == "RPOS"):
                
                received_msg = unpack(">fffx", received_msg)
                print(received_msg)

                print("\nWhat ASCII command to retreive info:\n1.RPOS\n2.OBSF\n3.RBID\n") # Print menu for client

            elif (msg == "OBSF"):
                
                received_msg = unpack(">ixxx", received_msg)[0]

                print(received_msg)

                print("\nWhat ASCII command to retreive info:\n1.RPOS\n2.OBSF\n3.RBID\n") # Print menu for client


            elif (msg == "RBID"):

                addr = unpack(">ixxx", received_msg)[0]
                addr = pack(">i",addr)
        
                received_msg = socket.inet_ntoa(addr)

                print(received_msg)

                print("\nWhat ASCII command to retreive info:\n1.RPOS\n2.OBSF\n3.RBID\n") # Print menu for client

            
            else:
                print(received_msg.decode(FORMAT)) # Print menu for client
                print("\nWhat ASCII command to retreive info:\n1.RPOS\n2.OBSF\n3.RBID\n")
        else:
            print("DISCONNECTED")
            
    except:
        print('[DISCONNECT] Disconnected from the server')




if __name__ == "__main__":

    connection()
    client.close()
