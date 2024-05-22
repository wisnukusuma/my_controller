import rclpy
from rclpy.node import Node
# from msgsTemplate.msg import MotorCommand
# from msgsTemplate.msg import MotorVels
# from msgsTemplate.msg import EncoderVals
from geometry_msgs.msg import Twist


import time
import math
import serial
from threading import Lock
import datetime



class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')
        # Setup parameters
        self.declare_parameter('loop_rate', value=0)
        if (self.get_parameter('loop_rate').value == 0):
            print("WARNING! LOOP RATE SET TO 0!!")
        self.led_pin_bootDone = 13

        self.declare_parameter('serial_port', value="/dev/ttyACM0")
        self.serial_port = self.get_parameter('serial_port').value


        self.declare_parameter('baud_rate', value=115200)
        self.baud_rate = self.get_parameter('baud_rate').value


        self.declare_parameter('serial_debug', value=False)
        self.debug_serial_cmds = self.get_parameter('serial_debug').value
        if (self.debug_serial_cmds):
            print("Serial debug enabled")
        self.declare_parameter('robot_simulation', value=False)
        self.robotSim =self.get_parameter('robot_simulation').value
        if (self.robotSim):
            print("Robot simulation enabled")
        # Setup topics & services
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.motor_command_callback,
            10)
        
        # Member Variables
        self.mutex = Lock()
        # Open serial comms
        if(self.robotSim == False):
            print(f"Connecting to port {self.serial_port} at {self.baud_rate}.")
            self.conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1.0)
            print(f"Connected to {self.conn}")
        

    # Raw serial commands
    
    def send_robot_vel_command(self, linear_vel, angular_vel):
        self.send_command(f"v {float(linear_vel)} {float(angular_vel)}")



    # def send_encoder_read_command(self):
    #     resp = self.send_command(f"e")
    #     if resp:
    #         return [int(raw_enc) for raw_enc in resp.split()]
    #     return []
    
    
    def map(x, in_min, in_max, out_min, out_max):
        out = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        return out 
    # More user-friendly functions
    #uncoment below for using stepper motor
    # Twist.linear.x is linear robot speed, Twist.angular.z is angular robot speed
    def motor_command_callback(self,Twist):
        if(Twist.linear.x or Twist.angular.z):
            if(self.robotSim == False):
                self.send_robot_vel_command(Twist.linear.x, Twist.angular.z)
        ts = datetime.datetime.now()
    #Get the battery status
        resp = int(self.send_command((f"g")))
        # print(len(resp))
        Battery =(resp - 453) * 100  / (514 - 453)
        print(str(ts)+' Battery ='+str(Battery)+'% =>'+str(Twist))
        # # print(str(ts)+' => '+str(Twist))
    
    # Utility functions

    def send_command(self, cmd_string):
        
        self.mutex.acquire()
        try:
            cmd_string += "\r"
            self.conn.write(cmd_string.encode("utf-8"))
            if (self.debug_serial_cmds):
                print("Sent: " + cmd_string)

            ## Adapted from original
            c = ''
            value = ''
            while c != '\r':
                c = self.conn.read(1).decode("utf-8")
                if (c == ''):
                    print("Error: Serial timeout on command: " + cmd_string)
                    return ''
                value += c

            value = value.strip('\r')

            if (self.debug_serial_cmds):
                print("Received: " + value)
            return value
        finally:
            self.mutex.release()

    def close_conn(self):
        self.conn.close()



def main(args=None):
    
    rclpy.init(args=args)

    motor_driver = MotorDriver()

    rate = motor_driver.create_rate(30)
    while rclpy.ok():
        rclpy.spin_once(motor_driver)
        # motor_driver.check_encoders()


    motor_driver.close_conn()
    motor_driver.destroy_node()
    rclpy.shutdown()


