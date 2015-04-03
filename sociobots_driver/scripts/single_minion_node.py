#!/usr/bin/env python

import rospy
import serial
import struct
import binascii
from geometry_msgs.msg import Twist
from xbee import ZigBee
import time

xbee = None
XBEE_ADDR_LONG = '\x00\x13\xA2\x00\x40\x31\x56\x46'
XBEE_ADDR_SHORT = '\x10\x52'
DEVICE = '/dev/ttyUSB0'


def print_data(data):
    rospy.loginfo("XBee Response: %s" % data)

def prepare_data(msg):
    linear = 0;
    angular = 0;

    if msg.linear.x > 0:
        linear = 1
    elif msg.linear.x < 0:
        linear = 2

    if msg.angular.z > 0:
        angular = 2
    elif msg.angular.z < 0:
        angular = 1

    data = struct.pack('BBB', 1, linear, angular)
    return data

def cmd_vel_command(data):
    #a,b,c
    #a = 1
    #b = 1 - forward, b = -1 - backward
    #c > 0 - turn right, c < 0 - turn left
    
    cmd = data
    print "left_Real" , cmd[0]
    print  "right_real", cmd[1]
    if (cmd[0] > 0):
        xbee.send('at',command='\x6D\x6D')
    else:
        xbee.send('at',command='\x6E\x6E')

    if (cmd[1] > 0 ):
        xbee.send('at',command='\x70\x70')
    else:
        xbee.send('at',command='\x71\x71')
    

    left=str(unichr(int(abs(cmd[0])*10)))
    right=str(unichr(int(abs(cmd[1])*10)))
  

    a= '\x6C'
    b= '\x72'
    
    print "left",left
    print "right",right
    
    xbee.send('at', command= a +left )
    time.sleep(.1)
 
    xbee.send('at', command= b +right )

def callback(msg):
    rospy.loginfo("Received a /cmd_vel message!")
    rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))
    wheel_sep_=2
    cmd = {}
    cmd[0] = msg.linear.x - msg.angular.z * (wheel_sep_) / 2
    cmd[1] = msg.linear.x + msg.angular.z * (wheel_sep_) / 2
    cmd_vel_command(cmd)
    
    
def listener():
    global xbee

    ser = serial.Serial(DEVICE, 9600)
    xbee = ZigBee(ser, callback=print_data)
    rospy.loginfo("xbee started")
    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaenously.
    rospy.init_node('cmd_vel_listener', anonymous=True)

    rospy.Subscriber("/sociobots/cmd_vel", Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    xbee.halt()
    ser.close()
        
if __name__ == '__main__':
    listener()
