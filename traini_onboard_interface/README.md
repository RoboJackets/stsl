# traini_onboard_interface

This package provides a node to run on the robot and expose ROS topics for controlling it and reading from its sensors.

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

### Cross Compilation & Packaging

In the `deployment` directory, run `run_all.sh` to cross compile the code for
the BeagleBone Black and package it for installation. Each step has it's own
run script in its respective folder if you want to only run one part of the
process.

### Installation

Running the cross compilation and packaging script mentioned above will generate
two files: `stsl_package.tar.gz` and `install_stsl.sh`. Copy both of these files
to the BeagleBone's home directory and run `install_stsl.sh`.

When installation is complete, restart the BeagleBone and the interface node
should start automatically.
