#!/usr/bin/env python

import socket

HEADER = 512
PORT = 65432
FORMAT = 'utf-8'
DISCONNECT_MESSAGE = "!DISCONNECT"
HOST = '127.0.0.1'
ADDR = (0, 0)

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

def send():

    print(client.recv(2048).decode(FORMAT)) # Print menu for client

    msg = None

    while msg != DISCONNECT_MESSAGE:

        msg = input()

        if( msg == "1" or msg == "2" or msg == "3" or msg == DISCONNECT_MESSAGE ):
            message = msg.encode(FORMAT)
            msg_length = len(message)
            send_length = str(msg_length).encode(FORMAT)
            send_length += b' ' * (HEADER - len(send_length))
            client.send(send_length)
            client.send(message)
            print(client.recv(2048).decode(FORMAT)) # Print menu for client
        else:
            print("[ERROR] Can't send this message. Try again")
    
    print("[DISCONNECT] Connection to server has been lost")

def connection():

    print("What IP adress would you like to connect to ?")

    while True:

        HOST = input()

        if HOST != 0xFFFF:
            ADDR = (HOST, PORT)
            break

    client.connect(ADDR)
    send()


if __name__=="__main__":

    connection()

