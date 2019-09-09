HOST = "localhost"
PORT = 4223
from tinkerforge.ip_connection import IPConnection
from tinkerforge.brick_silent_stepper import BrickSilentStepper
from tinkerforge.brick_stepper import BrickStepper



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

if __name__ == "__main__":
    ipcon = IPConnection() # Create IP connection
    current = 800
    max_velocity = 4000
    ramping_speed = 4000

    ipcon.connect(HOST, PORT) # Connect to brickd
    # Don't use device before ipcon is connected


    UID_l1 = "67QG3v" # Change XXYYZZ to the UID of your Silent Stepper Brick
    UID_l2 = "62fWRx" # Change XXYYZZ to the UID of your Silent Stepper Brick
    UID_r1 = "6JJW1Y" # Change XXYYZZ to the UID of your Silent Stepper Brick
    UID_r2 = "62YAyZ" # Change XXYYZZ to the UID of your Silent Stepper Brick

    motor_l = Motor_row( UID_l1, UID_l2,  ipcon, current, max_velocity, ramping_speed )

    motor_r = Motor_row( UID_r1, UID_r2,  ipcon, current, max_velocity, ramping_speed)

    motor_l.enable_motors()
    motor_r.enable_motors()

    motor_l.drive_forward()
    motor_r.drive_forward()

    motor_l.set_steps(30)
    motor_r.set_steps(30)

    import time 
    time.sleep(2)

    motor_l.disable()
    motor_r.disable()


    ipcon.disconnect()
