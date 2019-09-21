#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
rospy.loginfo("staring")
from tinkerforge.bricklet_led_strip_v2 import BrickletLEDStripV2
from tinkerforge.ip_connection import IPConnection

red = [0,255,0]*200
green = [0,0,255]*200
orange = [0, 255, 187]*200

HOST = "localhost"
PORT = 4223
UID = "FRw" # Change XYZ to the UID of your LED Strip Bricklet 2.0


def callback(data):
    nearest = min(data.ranges)
    if nearest < 0.3:
        rospy.loginfo("red")
        ls.set_led_values(0, red )
        return red
    if nearest < 0.4:
        rospy.loginfo("orange")
        ls.set_led_values(0, orange )
        return orange
    rospy.loginfo("green")
    ls.set_led_values(0, green )
    return green

def listener():

    rospy.init_node('waring_led', anonymous=True)

    rospy.Subscriber("/scan", LaserScan, callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    rospy.loginfo("staring")
    ipcon = IPConnection() # Create IP connection
    ls = BrickletLEDStripV2(UID, ipcon) # Create device object
    ipcon.connect(HOST, PORT) # Connect to brickd
    ls.set_chip_type(ls.CHIP_TYPE_WS2812)
    listener()
