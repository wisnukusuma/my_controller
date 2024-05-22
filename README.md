# ROS/Arduino Serial Motor Demo

This is demonstration of a ROS 2 interface to an Arduino running differential-drive motor control code.

The corresponding Arduino code can be found [here](https://github.com/joshnewans/ros_arduino_bridge), which is itself a fork of [this repo](https://github.com/hbrobotics/ros_arduino_bridge), which also contains a similar implementation for the ROS/Python/Client side (ROS 1 though).

## Components

The `controller` package consists of three nodes, `driver.py`, `odomTf.py` and `publsiherTest.py`. The idea is that the driver can be run on an onboard PC inside a robot (e.g. a Raspberry Pi), interfacing with the lower-level hardware. The driver exposes motor control through ROS topics (see below), which are to be published by the user's software.



## Driver configuration & usage

The driver has a few parameters:

- `loop_rate` - Execution rate of the *Arduino* code (see Arduino side documentation for details)
- `serial_port` - Serial port to connect to (default `/dev/ttyACM0`)
- `baud_rate` - Serial baud rate (default `115200`)
- `serial_debug` - Enables debugging of serial commands (default `false`)
- `robot_simulation` - Enables simulation of the robot(Twist message will not be forwarded through serial) (default `false`)

To run, e.g.
```
ros2 run controller [Nodes]
```
Nodes are driver,odomTf, publisherTest

for defined parameters run type following command (only for driver Node)
```
ros2 run controller driver --ros-args -p loop_rate:=30 -p serial_port:=/dev/ttyACM0 -p baud_rate:=115200 -p robot_simulation:=False
```





## GUI Usage

Has two modes, one for raw PWM input (-255 to 255) and one for closed-loop control. In this mode you must first set the limits for the sliders.


## TODO

- Add service for encoder reset
- Add service for updating PID parameters
- Stability improvements
- More parameterisation



