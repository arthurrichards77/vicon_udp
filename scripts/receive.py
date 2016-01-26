#!/usr/bin/env python

import socket
import struct

UDP_IP = "192.168.10.81"
UDP_PORT = 27002

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

while True:
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    print "received message:", data
    print data[0:2]=='VI'
    chksum1 = struct.unpack('I', data[35:39])
    chksum2 = sum(bytearray(data[0:35]))
    print chksum1[0]==chksum2
    (x,y,z,qx,qy,qz,qw) = struct.unpack('fffffff', data[6:34])
    print x,y,z,qx,qy,qz,qw
    
