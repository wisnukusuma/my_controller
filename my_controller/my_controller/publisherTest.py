import rclpy
from rclpy.node import Node
import time
import math
from geometry_msgs.msg import Twist


class PubTest(Node):

    def __init__(self):
        super().__init__('publisherTest')

        self.publisher = self.create_publisher(Twist,'cmd_vel', 10)

        self.create_timer(0.1,self.send_motor_once)

    def send_motor_once(self):
        msg = Twist()
        msg.linear.x = 50.0
        msg.angular.z = 0.0
        
        self.publisher.publish(msg)


def main(args=None):
    
    rclpy.init(args=args)

    robotTest = PubTest()

    rate = robotTest.create_rate(100)    
    while rclpy.ok():
        rclpy.spin_once(robotTest)
    

    robotTest.destroy_node()
    rclpy.shutdown()


