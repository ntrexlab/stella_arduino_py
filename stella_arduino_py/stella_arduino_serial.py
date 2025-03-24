#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Bool
import serial
import time

class STELLA_Arduino(Node):

    def __init__(self):
        self.serial_port = serial.Serial(
            port="/dev/ttyACM0",
            baudrate=9600,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
        
        time.sleep(1)
        
        super().__init__('stella_arduino_serial')
        
        self.subscription = self.create_subscription(
            Bool,
            'light_on',
            self.listener_callback,
            10
        )
        
        self.pub = self.create_publisher(Int16, 'CDS_brightness', 10)
        
        timer_period = 0.1
        
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def listener_callback(self, msg: Bool):
        try:
            if msg.data:
                self.serial_port.write(b'\x01') 
                self.get_logger().info('Light ON command sent')
            else:
                self.serial_port.write(b'\x00') 
                self.get_logger().info('Light OFF command sent')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial error: {e}')
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {e}')  
  
    def timer_callback(self):
        try:
            line = self.serial_port.readline().decode('utf-8').strip()
            if line:
                brightness_value = int(line)
                msg = Int16()
                msg.data = brightness_value
                self.pub.publish(msg)
                self.get_logger().info(f'Published brightness: {brightness_value}')
        except ValueError:
            self.get_logger().warn(f'Invalid data received: {line}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial error: {e}')
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {e}')
    
def main(args=None):
  
    rclpy.init(args=args)
    arduino = STELLA_Arduino()
    try:
        rclpy.spin(arduino)
    except KeyboardInterrupt:
        pass
    # Exception handling to remove unnecessary logs
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        arduino.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception as e:
                pass
        else:
            pass
  
if __name__ == '__main__':
  main()

