import math

# from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16


import numpy as np
import time

from math import sin, cos, pi
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, TransformStamped
from nav_msgs.msg import Odometry
#

x = 0.0
y = 0.0
th = 0.0

vx = 0
vy = 0
vth = 0

NS_TO_SEC= 1000000000

# def quaternion_from_euler(ai, aj, ak):
#     ai /= 2.0
#     aj /= 2.0
#     ak /= 2.0
#     ci = math.cos(ai)
#     si = math.sin(ai)
#     cj = math.cos(aj)
#     sj = math.sin(aj)
#     ck = math.cos(ak)
#     sk = math.sin(ak)
#     cc = ci*ck
#     cs = ci*sk
#     sc = si*ck
#     ss = si*sk

#     q = np.empty((4, ))
#     q[0] = cj*sc - sj*cs
#     q[1] = cj*ss + sj*cc
#     q[2] = cj*cs - sj*sc
#     q[3] = cj*cc + sj*ss

#     return q

class FramePublisher(Node):

    def __init__(self):
        super().__init__('odomTf')
        self.nodename = "odomTf"
        self.get_logger().info(f"-I- {self.nodename} started")

        #### parameters #######
        self.odomRate_hz = self.declare_parameter("rate_hz", 10.0).value # the rate at which to publish the transform
        self.create_timer(1.0/self.odomRate_hz, self.update) 
        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        # self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))
        # loop_rate = self.create_rate(30)

        self.ticks_meter = float(
            self.declare_parameter('ticks_meter', 6400).value)  # The number of wheel encoder ticks per meter of travel
        self.base_width = float(self.declare_parameter('base_width', 0.135).value)  # The wheel base width in meters

        self.base_frame_id = self.declare_parameter('base_frame_id',
                                                    'base_link').value  # the name of the base frame of the robot
        self.odom_frame_id = self.declare_parameter('odom_frame_id',
                                                    'odom').value  # the name of the odometry reference frame

        self.encoder_min = self.declare_parameter('encoder_min', -2147483648).value
        self.encoder_max = self.declare_parameter('encoder_max', 2147483647).value
        self.encoder_low_wrap = self.declare_parameter('wheel_low_wrap', (
                self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min).value
        self.encoder_high_wrap = self.declare_parameter('wheel_high_wrap', (
                self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min).value

        # internal data
        self.enc_left = None  # wheel encoder readings
        self.enc_right = None
        self.left = 0.0  # actual values coming back from robot
        self.right = 0.0
        self.lmult = 0.0
        self.rmult = 0.0
        
        self.linVel = 0.0
        self.angVel = 0.0

        self.joint_state = JointState()
        self.lwheel = 0.0
        self.rwheel = 0.0
        self.speedR = 0.0
        self.speedL = 0.0
        # self.numStepR= 0.0
        # self.numStepL= 0.0

        self.x = 0.0  # position in xy plane
        self.y = 0.0
        self.th = 0.0
        self.dx = 0.0  # speeds in x/rotation
        self.dr = 0.0
        

        #Motor Specification
        self.motorStep = 6400
        self.wheelCirc = 21.5
        self.MAXIMUM_SPEED = 20 # cm/s
        self.MINIMUM_SPEED = 2 # cm/s

        self.ANG_MAXIMUM_SPEED = 2.0 # rad/s
        # subscriptions
        # self.create_subscription(Int16, "lwheel", self.lwheel_callback, 10)
        # self.create_subscription(Int16, "rwheel", self.rwheel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.odom_broadcaster = TransformBroadcaster(self,qos=qos_profile)

        # Declare and acquire `turtlename` parameter
        self.robotname = self.declare_parameter(
          'kalBot', 'mobile_robot').get_parameter_value().string_value
        print(self.robotname+" initializing")

        # Initialize the transform broadcaster
        # self.tf_broadcaster = TransformBroadcaster(self)

        # callback function on each message
        self.subscription = self.create_subscription(
             Twist,
            'cmd_vel',
            self.speedUpdate,
            10)
        self.subscription  # prevent unused variable warning
        self.then = self.get_clock().now()
        
    def speedUpdate(self, msg): 
        self.linVel= msg.linear.x
        self.angVel=msg.angular.z
    # def tf_update(self, msg):
    #     t = TransformStamped()
    #     global  th , x , y , vx , vth , vy , dt
    #     # Read message content and assign it to
    #     # corresponding tf variables
    #     t.header.stamp = self.get_clock().now().to_msg()
    #     t.header.frame_id = 'world'
    #     t.child_frame_id = self.robotname

    #     vx = msg.linear.x * math.cos(th) *0.01
    #     vy = msg.linear.x * math.sin(th) *0.01
    #     vth = msg.angular.z 
        
    #     # times 1 because a command in cm/s
    #     delta_x = vx * 1
    #     delta_y = vy * 1
    #     delta_th = vth * 1 

    #     x += delta_x
    #     y += delta_y
    #     th += delta_th  

    #     # Turtle only exists in 2D, thus we get x and y translation
    #     # coordinates from the message and set the z coordinate to 0
    #     t.transform.translation.x = x
    #     t.transform.translation.y = y
    #     t.transform.translation.z = 0.0

    #     # For the same reason, turtle can only rotate around one axis
    #     # and this why we set rotation in x and y to 0 and obtain
    #     # rotation in z axis from the message
    #     q = quaternion_from_euler(0, 0, th)
    #     t.transform.rotation.x = q[0]
    #     t.transform.rotation.y = q[1]
    #     t.transform.rotation.z = q[2]
    #     t.transform.rotation.w = q[3]
    #     print(msg)
    #     # Send the transformation
    #     self.tf_broadcaster.sendTransform(t)

    def update(self):
            #calculate wheel TF
            self.speedR=self.linVel+(self.angVel* 6.75)
            self.speedL=self.linVel-(self.angVel* 6.75)
            if(self.speedR != 0):
                if(abs(self.speedR)<self.MINIMUM_SPEED):
                    if(self.speedR<0):
                        self.speedR=-self.MINIMUM_SPEED
                    else:
                        self.speedR=self.MINIMUM_SPEED
                elif(abs(self.speedR)>self.MAXIMUM_SPEED):
                    if(self.speedR<0):
                        self.speedR=-self.MAXIMUM_SPEED
                    else:
                        self.speedR=self.MAXIMUM_SPEED 
            
            if(self.speedL != 0):
                if(abs(self.speedL)<self.MINIMUM_SPEED):
                    if(self.speedL<0):
                        self.speedL=-self.MINIMUM_SPEED
                    else:
                        self.speedL=self.MINIMUM_SPEED
                elif(abs(self.speedL)>self.MAXIMUM_SPEED):
                    if(self.speedL<0):
                        self.speedL=-self.MAXIMUM_SPEED
                    else:
                        self.speedL=self.MAXIMUM_SPEED 
            
            lVel = 0.0
            if(self.linVel != 0):
                if(abs(self.linVel)<self.MINIMUM_SPEED):
                    if(self.linVel<0):
                        lVel=-self.MINIMUM_SPEED
                    else:
                        lVel=self.MINIMUM_SPEED
                elif(abs(self.linVel)>self.MAXIMUM_SPEED):
                    if(self.linVel<0):
                        lVel=-self.MAXIMUM_SPEED
                    else:
                        lVel=self.MAXIMUM_SPEED

            aVel = 0.0
            if(self.angVel != 0):
                if(abs(self.angVel)>self.MAXIMUM_SPEED):
                    if(self.angVel<0):
                        aVel=-self.ANG_MAXIMUM_SPEED
                    else:
                        aVel=self.ANG_MAXIMUM_SPEED 
                else:
                    aVel = self.angVel  

            
            numStepR=self.speedR*(self.motorStep/self.wheelCirc)
            numStepL=self.speedL*(self.motorStep/self.wheelCirc)
            now = self.get_clock().now()
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.nanoseconds / NS_TO_SEC
            if(self.speedR == 0):
                stepDelayR = 0
            else:    
                stepDelayR=(1/numStepR)*0.5
                self.rwheel+=(elapsed/stepDelayR)
            
            if(self.speedL == 0):
                stepDelayL = 0
            else:
                stepDelayL=(1/numStepL)*0.5
                self.lwheel+=(elapsed/stepDelayL)
            
            self.joint_state.header.stamp = now.to_msg()
            self.joint_state.name = ['right_wheel_joint', 'left_wheel_joint']
            self.joint_state.position = [self.rwheel, self.lwheel]
            self.joint_pub.publish(self.joint_state)
            # print("rwheel= "+str(self.rwheel)+" ,lwheel= "+str(self.lwheel))
            # print("rSpeed= "+str(self.speedR)+" ,lSpeed= "+str(self.speedL))
            # calculate odometry
            # if self.enc_left == None or self.enc_right == None:
            #     d_left = 0
            #     d_right = 0
            # else:
            #     d_left = (self.left - self.enc_left) / self.ticks_meter
            #     d_right = (self.right - self.enc_right) / self.ticks_meter
            # self.enc_left = self.lwheel
            # self.enc_right = self.rwheel

            # distance traveled is the average of the two wheels 
            d = ( self.rwheel + self.lwheel) / 2
            # this approximation works (in radians) for small angles
            # th = (self.rwheel - self.lwheel) / self.base_width
            # calculate velocities
            # self.dx = d / elapsed
            # self.dr = th / elapsed

            self.dx = self.linVel
            self.dr = self.angVel


            self.th = self.th + (aVel/10.0)  
            x = cos(self.th) * (lVel/100.0)
            y = sin(self.th) * (lVel/100.0)
                
            # calculate the final position of the robot
            self.x = self.x + x
            self.y = self.y + y

            # if d != 0:
            #     # calculate distance traveled in x and y
            #     x = cos(aVel) * (lVel/100.0)
            #     y = sin(aVel) * (lVel/100.0)
                
            #     # calculate the final position of the robot
            #     self.x = self.x + (cos(self.th) * x - sin(self.th) * y)
            #     self.y = self.y + (sin(self.th) * x + cos(self.th) * y)
            # if th != 0:
            #     self.th = self.th + aVel
            
            # print("Linear= "+str(self.linVel)+" ,Angular= "+str(self.th))
            # print("x= "+str(self.x)+" ,y= "+str(self.y))

            # publish the odom information
            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin(self.th / 2)
            quaternion.w = cos(self.th / 2)

            transform_stamped_msg = TransformStamped()
            transform_stamped_msg.header.stamp = self.get_clock().now().to_msg()
            transform_stamped_msg.header.frame_id = self.odom_frame_id
            transform_stamped_msg.child_frame_id = self.base_frame_id
            transform_stamped_msg.transform.translation.x = self.x
            transform_stamped_msg.transform.translation.y = self.y
            transform_stamped_msg.transform.translation.z = 0.0
            transform_stamped_msg.transform.rotation.x = quaternion.x
            transform_stamped_msg.transform.rotation.y = quaternion.y
            transform_stamped_msg.transform.rotation.z = quaternion.z
            transform_stamped_msg.transform.rotation.w = quaternion.w

            self.odom_broadcaster.sendTransform(transform_stamped_msg)

            odom = Odometry()
            odom.header.stamp = now.to_msg()
            odom.header.frame_id = self.odom_frame_id
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation = quaternion
            odom.child_frame_id = self.base_frame_id
            odom.twist.twist.linear.x = self.dx
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.angular.z = self.dr
            self.odom_pub.publish(odom)



def main():
    rclpy.init()
    try:
        node = FramePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    