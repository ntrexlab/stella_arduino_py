# STELLA Arduino ROS 2 Package

This package provides a simple ROS 2 example for communicating with an Arduino device over a serial connection.

This package is designed for use with Arduino boards that do not support micro-ROS.

It demonstrates reading brightness data from a light sensor and controlling an Arduino's built-in LED based on that data using ROS 2 topics. 

This example serves as a basic template that can be easily adapted to interface with various other sensors and actuators connected to an Arduino.

## Package Overview

This package contains the following ROS 2 nodes and directories:

- **`stella_arduino_serial`**: 
  - Reads data from a serial-connected sensor (in this example, a CDS light sensor).
  - Publishes the sensor data as a `std_msgs/msg/Int16` message on the `/CDS_brightness` topic.
  - Listens to the `/light_on` topic (`std_msgs/msg/Bool`) and sends ON/OFF commands (`1\n` or `0\n`) back to the Arduino via serial to control an actuator (in this example, the built-in LED).

- **`stella_arduino_node`**: 
  - Subscribes to `/CDS_brightness` and determines whether to activate the actuator based on a predefined threshold (e.g., 600 for brightness).
  - Publishes a `Bool` message on the `/light_on` topic.

- **`test/`**:
  - This directory contains Python scripts for testing the serial communication using the `pyserial` library.

- **`arduino/`**:
  - This directory contains the Arduino sketch (`.ino` file) used in this example. It includes the code for reading data from the light sensor and controlling the built-in LED based on commands received over the serial port.

## Dependencies

- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/index.html)
- Python 3.7+
- [pyserial](https://pypi.org/project/pyserial/)

## Installation
Clone this package into your ROS 2 workspace:

```bash
cd ~/colcon_ws/src
git clone https://github.com/ntrexlab/stella_arduino_py.git
cd ..
colcon build --packages-select stella_arduino_py
source install/local_setup.bash
```
Check that the Arduino is connected to a valid serial port (e.g., /dev/ttyACM0). if not, you need to change the serial port in **stella_arduino_serial.py**.


## Running the Nodes
1. Start the serial communication node (reading sensor data & sending control data):
```bash
ros2 run stella_arduino_py stella_arduino_serial
```
2. Start the control logic node (decision-making based on sensor data):
```bash
ros2 run stella_arduino_py stella_arduino_node
```
## License
This project is licensed under the Apache License 2.0.

## Maintainer
NTREX LAB
lab@ntrex.co.kr
