#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import Twist, Vector3
import struct
from tinkerforge.ip_connection import IPConnection
from tinkerforge.brick_silent_stepper import BrickSilentStepper

HOST = "localhost"
PORT = 4223
UID = "XXYYZZ" # Change XXYYZZ to the UID of your Servo Brick



speed = 0
direction = 0
timeout = 0
ser = None

TIMEOUT      = 10
MAX_SPEED    = 1 # m/s
MAX_STEERING = 0.1 # rad/s

import threading



def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

def callback(data):
    global speed, direction, timeout
    speed = clamp(data.linear.x, -MAX_SPEED, MAX_SPEED)
    direction = clamp(data.angular.z, -MAX_STEERING, MAX_STEERING)
    timeout = 0
    #rospy.loginfo("Speed: %f", speed)

def main():
    global speed, direction, timeout, ser
    rospy.init_node('ros_stepper')

    ipcon = IPConnection() # Create IP connection
    servo = BrickServo(UID, ipcon) # Create device object
    ipcon.connect(HOST, PORT)

    rospy.Subscriber("/cmd_vel", Twist, callback)
    r = rospy.Rate(100) # Hz
    vel_tp = [0] * 50 # 50 sample low-pass for speed
    dir_tp = [0] * 10 # 10 sample low-pass for steering

  



    while not rospy.is_shutdown():
        vel_tp[len(vel_tp)-1] = speed #if not timeout > TIMEOUT else 0
        vel_tp[:-1] = vel_tp[1:]

        dir_tp[len(dir_tp)-1] = direction
        dir_tp[:-1] = dir_tp[1:]

        tx_speed = (sum(vel_tp)/len(vel_tp))
        tx_dir = (sum(dir_tp)/len(dir_tp))

        rospy.loginfo("Speed: %f", tx_speed)
        rospy.loginfo("Steering: %f", tx_dir)

        motorR = tx_speed + tx_dir
        motorL= tx_speed - tx_dir

        rospy.loginfo("MotorR: %f", motorR)
        rospy.loginfo("MotorL: %f", motorL)
        
        

        timeout+=1
        r.sleep()

main()
