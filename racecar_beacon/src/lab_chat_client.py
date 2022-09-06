#!/usr/bin/env python

from encodings import utf_8
import socket

HOST = '10.42.0.211'
PORT = 65432

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

def sendMessage(msg):
    message = msg.encode(utf_8)

s.send('disconnect'.encode('utf_8'))
data = s.recv(1024)
print(data)
s.close()


