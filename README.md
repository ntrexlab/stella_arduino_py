# STELLA Arduino ROS 2 Package

This package provides a simple ROS 2 interface for communicating with an Arduino device over a serial connection. It reads brightness data from a light sensor and controls a light based on that data using ROS 2 topics.

## Package Overview

This package contains the following ROS 2 nodes:

- **`stella_arduino_serial`**: 
  - Reads brightness values from a serial-connected light sensor (CDS).
  - Publishes the brightness as a `std_msgs/msg/Int16` message on the `/CDS_brightness` topic.
  - Listens to the `/light_on` topic (`std_msgs/msg/Bool`) and sends ON/OFF commands (`1\n` or `0\n`) back to the Arduino via serial.

- **`stella_arduino_node`**: 
  - Subscribes to `/CDS_brightness` and determines whether to turn the light on or off based on a brightness threshold (e.g., 600).
  - Publishes a `Bool` message on the `/light_on` topic.

## Dependencies

- ROS 2 (Jazzy)
- Python 3.7+
- [`pyserial`](https://pypi.org/project/pyserial/)

Install `pyserial` with:

```bash
pip install pyserial
```
## Installation
Clone this package into your ROS 2 workspace:

```bash
cd ~/colcon_ws/src
git clone https://github.com/ntrexlab/stella_arduino_py.git
cd ..
colcon build --packages-select stella_arduino_py
source install/local_setup.bash
```
Make sure the Arduino is connected to a valid serial port (e.g., /dev/ttyACM0) and is sending brightness values (as strings with newline \n).

## Running the Nodes
1. Start the serial node (sensor & control):
```bash
ros2 run stella_arduino_py stella_arduino_serial
```
2. Start the control logic node (decision-making):
```bash
ros2 run stella_arduino_py stella_arduino_node
```
## License
This project is licensed under the Apache License 2.0.

## Maintainer
NTREX LAB
lab@ntrex.co.kr
