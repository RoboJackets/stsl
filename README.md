# Software Training Support Library (STSL)

[![Maintainer](https://img.shields.io/badge/Maintainer-Matthew%20Barulic-blue.svg)](https://github.com/barulicm)

The STSL is a library which facilitates the RoboJackets software training program. It has taken many forms over the years as the hardware platforms have changed.

Currently, the STSL is a ROS 2 package providing an interface for controlling custom robots based on the Beaglebone Black Wireless.

## robot_interface_node

### Published Topics

- **/ultrasonic** ( std_msgs/Float32 )

   Distance, in meters, as measured by the ultrasonic sensor. -1.0 indicates a measurement failure.
   
- **/encoder_left** ( std_msgs/UInt64 )

   Position of the left wheel in encoder ticks (40 ticks / rotation).

- **/encoder_right** ( std_msgs/UInt64 )

    Position of the right wheel in encoder ticks (40 ticks / rotation).

- **/line_center** ( std_msgs/UInt64 )

    Raw light measurement value from the center line sensor.

- **/line_side** ( std_msgs/UInt64 )

    Raw light measurement value from the side line sensor.

### Subscribed Topics

- **/motor_left** ( std_msgs/Float32 )

    Power to provide to the left motor. Range -1.0 to 1.0. Positive numbers roll forward, negative numbers roll backwards.

- **/motor_right** ( std_msgs/Float32 )

    Power to provide to the left motor. Range -1.0 to 1.0. Positive numbers roll forward, negative numbers roll backwards.

## Deploying

### Cross Compilation

TODO: Instructions coming soon

### Packaging

TODO: Instructions coming soon

### Installation

TODO: Instructions coming soon
