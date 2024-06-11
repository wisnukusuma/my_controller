import rclpy
from rclpy.node import Node
from rclpy.time import Time 
# from msgsTemplate.msg import MotorCommand
# from msgsTemplate.msg import MotorVels
# from msgsTemplate.msg import EncoderVals
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Range

import time
import math
import serial
from threading import Lock
import datetime

start = Time()
rangeMsg = Range()
scanMsg = LaserScan()

class Sonar(Node):
    def __init__(self):
        super().__init__('sonar_driver')
        # Setup parameters
        

        self.declare_parameter('serial_port', value="/dev/ttyUSB0")
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
        
        self.buffScan = [ 0.0 , 0.0 , 0.0]
        scanMsg.header.frame_id = "sonar_link"
        scanMsg.angle_min =  -0.125
        scanMsg.angle_max =  0.125
        scanMsg.angle_increment = 0.1
        scanMsg.time_increment = 0.0
        scanMsg.range_min = 0.05
        scanMsg.range_max = 2.0

        self.buffRange = [ 0.0 , 0.0 , 0.0]
        rangeMsg.header.frame_id = "sonar_link"
        rangeMsg.radiation_type = 0
        rangeMsg.min_range = 0.05
        rangeMsg.max_range = 0.60
        rangeMsg.field_of_view = 0.17   # 10deg

        # Setup topics & services



        # Member Variables
        self.mutex = Lock()
        # Open serial comms
        if(self.robotSim == False):
            print(f"Connecting to port {self.serial_port} at {self.baud_rate}.")
            self.conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1.0)
            print(f"Connected to {self.conn}") 
          
            self.publisher = self.create_publisher(LaserScan,'scan', 10)
            self.create_timer(0.1,self.scan_callback)

            # self.publisher = self.create_publisher(Range,'range', 10)
            # self.create_timer(1,self.range_callback)
    
    

    
    def map(x, in_min, in_max, out_min, out_max):
        out = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        return out 

    # Utility functions
    def scan_callback(self):
        msgBuff = self.send_command(f"h")
        if (msgBuff == ''):
            return
        else:
            self.buffScan[0]=self.buffScan[1]
            self.buffScan[1]=self.buffScan[2]
            self.buffScan[2]= float(msgBuff)
       
        if(self.buffScan[0] > 0.0 and self.buffScan[1] > 0.0 and self.buffScan[2] > 0.0):
            scanMsg.ranges = [self.buffScan[0] / 100,self.buffScan[0] / 100,self.buffScan[0] / 100,]
            scanMsg.intensities = [0.0 , 0.0 , 0.0 ]
            # print(self.get_clock().now().to_msg())
            scanMsg.header.stamp = self.get_clock().now().to_msg()

            self.publisher.publish(scanMsg)
        else:
            scanMsg.ranges = [0.0 , 0.0 , 0.0 ]
            scanMsg.intensities = [0.0 , 0.0 , 0.0 ]
            scanMsg.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(scanMsg)


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

    sonar = Sonar()

    rate = sonar.create_rate(30)
    while rclpy.ok():
        rclpy.spin_once(sonar)
        


    sonar.close_conn()
    sonar.destroy_node()
    rclpy.shutdown()


