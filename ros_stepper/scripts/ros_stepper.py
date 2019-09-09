#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import Twist, Vector3
import struct





speed = 0
direction = 0
timeout = 0
ser = None

TIMEOUT      = 10
MAX_SPEED    = 1 # m/s
MAX_STEERING = 0.1 # rad/s

UID_l1 = "67QG3v" # Change XXYYZZ to the UID of your Silent Stepper Brick
UID_l2 = "62fWRx" # Change XXYYZZ to the UID of your Silent Stepper Brick
UID_r1 = "6JJW1Y" # Change XXYYZZ to the UID of your Silent Stepper Brick
UID_r2 = "62YAyZ" # Change XXYYZZ to the UID of your Silent Stepper Brick


import threading



class Motor_row():

    def __init__(self,UID_front, UID_back, ipcon, current, max_velocity, ramping_speed ):
    
        self.UID_back = UID_back
        self.UID_front = UID_front
        # init both stepper
        self.motor_back = BrickStepper(UID_back, ipcon)
        self.motor_front = BrickSilentStepper(UID_front, ipcon)
        # init current in mA
        print(current)
        self.motor_back.set_motor_current(current)
        self.motor_front.set_motor_current(current)
        # init max_velocity
        print(max_velocity)
        self.motor_back.set_max_velocity(max_velocity)
        self.motor_front.set_max_velocity(max_velocity)
        # init ramping speed
        print(ramping_speed)
        self.motor_back.set_speed_ramping(ramping_speed, ramping_speed)
        self.motor_front.set_speed_ramping(ramping_speed, ramping_speed)

        self.motor_back.set_step_mode(8)
        self.motor_front.set_step_configuration(self.motor_front.STEP_RESOLUTION_8, True)

    def enable_motors(self):
        self.motor_back.enable()
        self.motor_front.enable()    

    def set_steps(self, steps):
        print(steps)
        self.motor_back.set_steps(steps)
        self.motor_front.set_steps(steps)

    def drive_forward(self):
        self.motor_back.drive_forward()
        self.motor_front.drive_forward()
    

    def drive_backward(self):
        self.motor_back.drive_backward()
        self.motor_front.drive_backward()

    def full_stop(self):
        self.motor_back.full_stop()
        self.motor_front.full_stop()

    def disable(self):
        self.motor_back.disable()
        self.motor_front.disable()





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
    current = 800
    max_velocity = 4000
    ramping_speed = 4000
    motor_umfang = 0.188496

    ipcon.connect(HOST, PORT) # Connect to brickd

    stepper_l = Motor_row(UID_l1, UID_r1, ipcon, current, max_velocity, ramping_speed)

    stepper_r = Motor_row(UID_l1, UID_r1, ipcon, current, max_velocity, ramping_speed)

    rospy.Subscriber("/cmd_vel", Twist, callback)
    r = rospy.Rate(100) # Hz
    vel_tp = [0] * 50 # 50 sample low-pass for speed
    dir_tp = [0] * 10 # 10 sample low-pass for steering

    stepper_l.enable_motors()

    stepper_r.enable_motors()



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
        motorL = tx_speed - tx_dir

        rospy.loginfo("MotorR: %f", motorR)
        rospy.loginfo("MotorL: %f", motorL)
        
        steps_motor_l =  motorL / motor_umfang * 8
        steps_motor_r =   motorR / motor_umfang * 8

        rospy.loginfo("MotorR S/s %f", steps_motor_r)
        rospy.loginfo("MotorL S/s %f", steps_motor_l)

        if stepper_l > 0:
            stepper_l.drive_forward()
        else:
            stepper_l.drive_backward()
        
        
        if stepper_r > 0:
            stepper_r.drive_forward()
        else:
            stepper_r.drive_backward()

        stepper_r.steps(abs(steps_motor_r))
        stepper_l.steps(abs(steps_motor_l))
        timeout+=1
        r.sleep()

    stepper_l.disable()
    stepper_r.disable()

main()
