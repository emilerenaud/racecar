#!/usr/bin/env python

import socket
from telnetlib import IP
from struct import *

HOST = '127.0.0.1'
PORT = 65432
# This process should listen to a different port than the PositionBroadcast client.

def menu():
    print("---------- Remote Client ----------")
    print("Options :")
    print("     1- Position of the robot")
    print("     2- Obstacle near the robot")
    print("     3- Robot ID")
    print("")
    menuSelection = input("Choose an option : ")
    return menuSelection


if __name__=="__main__":

    #get IP adress of the server
    try:
        print("---------- Remote Client ----------")
        IP_SERVER = input('IP adress of the server : ')
        if len(IP_SERVER) == 0:
            print('default one is 127.0.0.1')
            IP_SERVER = '127.0.0.1'
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((IP_SERVER, PORT))
        s.settimeout(1)
        # print(socket.gethostname())
        print('Connected to the server')
    except:
        print('Error while connecting with the server')
        exit()
        pass

    while True:
        menuSelection = menu()
        if menuSelection != '1' and menuSelection != '2' and menuSelection != '3':
            print("Choose a real option dumbass")
        else:
            if menuSelection == '1':
                msg = 'RPOS'.encode('utf-8')
            elif menuSelection == '2':
                msg = 'OBSF'.encode('utf-8')
            elif menuSelection == '3':
                msg = 'RBID'.encode('utf-8')

            try:
                s.send(msg)
            except:
                print("Error while sending a request")

            # Wait for the message.
            try:
                dataReceived, addr = s.recvfrom(1024)
                print("Received data from",addr," : ",dataReceived) #addr doesnt work.
                # data_unpack = unpack("fffI",data)
                # print("received message: ",data_unpack)
            except:
                pass
            try:
                if menuSelection == '1':
                    data_unpack = unpack("fffx",dataReceived)
                if menuSelection == '2':
                    data_unpack = unpack("Ixxx",dataReceived)
                if menuSelection == '3':
                    data_unpack = unpack("Ixxx",dataReceived)
                print("received message: ",data_unpack)
            except:
                print("Error while depacking.")

