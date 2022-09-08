#!/usr/bin/env python

from ast import Break
import socket
import threading

HOST = '127.0.0.1'
PORT = 65432


def receive(conn):
    while True:
        data = conn.recv(1024)
        if not data :
            None
        else:
            print(data)
    None

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
    s.bind((HOST, PORT))
    s.listen(2)
except:
    print("erreur while bind or listen")

(conn,addr) = s.accept()

while True:
    data = conn.recv(1024)

    if  not data :
        None
    else:
        print(data)
        conn.send(data)
    

    

conn.close()