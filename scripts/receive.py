#!/usr/bin/env python

import roslib
roslib.load_manifest('vicon_udp')
import rospy
import socket
import struct
import tf
from geometry_msgs.msg import TransformStamped

rospy.init_node('vicon_udp')
world_name = rospy.get_param('vicon_base_frame',"world")
object_name = rospy.get_param('~object_name',"thing")
tf_name = object_name + "/" + object_name

UDP_IP = rospy.get_param('own_ip',"127.0.0.1")
UDP_PORT = rospy.get_param('~udp_port',27001)

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

scale_factor = rospy.get_param('vicon_scale_factor',0.001)

br = tf.TransformBroadcaster()
pb = rospy.Publisher("vicon/"+tf_name, TransformStamped) #, queue_size=10)

while not rospy.is_shutdown():
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    #print "received message:", data
    #print data[0:2]=='VI'
    chksum1 = struct.unpack('I', data[35:39])
    chksum2 = sum(bytearray(data[0:35]))
    #print chksum1[0]==chksum2
    if chksum1[0]==chksum2:
        (x,y,z,qx,qy,qz,qw) = struct.unpack('fffffff', data[6:34])
        x = scale_factor*x
        y = scale_factor*y
        z = scale_factor*z
        #print x,y,z,qx,qy,qz,qw
        # send the pose
        tr = TransformStamped()
        tr.header.frame_id = world_name
        tr.header.stamp = rospy.Time.now()
        tr.child_frame_id = tf_name
        tr.transform.translation.x = x
        tr.transform.translation.y = y
        tr.transform.translation.z = z
        tr.transform.rotation.x = qx
        tr.transform.rotation.y = qy
        tr.transform.rotation.z = qz
        tr.transform.rotation.w = qw
        pb.publish(tr)
        # send TF
        br.sendTransform((x,y,z),(qx,qy,qz,qw),rospy.Time.now(),tf_name,world_name)
        #print "Sent TF"
                     
    
