import math

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
import numpy as np
import time
import math

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
x = 0.0
y = 0.0
th = 0.0

vx = 0
vy = 0
vth = 0


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class FramePublisher(Node):

    def __init__(self):
        super().__init__('kalBot_tf2_frame_publisher')

        # Declare and acquire `turtlename` parameter
        self.robotname = self.declare_parameter(
          'kalBot', 'mobile_robot').get_parameter_value().string_value
        print(self.robotname+" initializing")
        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
        # callback function on each message
        self.subscription = self.create_subscription(
             Twist,
            'cmd_vel',
            self.tf_update,
            10)
        self.subscription  # prevent unused variable warning

    def tf_update(self, msg):
        t = TransformStamped()
        global  th , x , y , vx , vth , vy , dt
        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = self.robotname

        vx = msg.linear.x * math.cos(th) *0.01
        vy = msg.linear.x * math.sin(th) *0.01
        vth = msg.angular.z 
        
        # times 1 because a command in cm/s
        delta_x = vx * 1
        delta_y = vy * 1
        delta_th = vth * 1 

        x += delta_x
        y += delta_y
        th += delta_th  

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        q = quaternion_from_euler(0, 0, th)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        print(msg)
        # Send the transformation
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()