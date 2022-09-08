#!/usr/bin/env python

from encodings import utf_8
import socket

HOST = '127.0.0.1'
PORT = 65432

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

def sendMessage(msg):
    message = msg.encode('utf_8')
    s.send(message)

sendMessage("HelloWord")
data = s.recv(1024)
print(data)
sendMessage("Hello 2")
s.close()
