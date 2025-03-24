#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Bool

class STELLA_Arduino_Controller(Node):

  def __init__(self):      
    super().__init__('stella_arduino_node')
      
    self.pub = self.create_publisher(Bool, 'light_on', 10)

    self.sub = self.create_subscription(
      Int16,
      'CDS_brightness',
      self.listener_callback,
      10
    )
    
  def listener_callback(self, msg: Int16):
    msg_light = Bool()

    if msg.data >= 600:
      msg_light.data = True
      self.get_logger().info('Light ON')
    else:
      msg_light.data = False
      self.get_logger().info('Light OFF')
    
    self.pub.publish(msg_light)
    
def main(args=None):
  
    rclpy.init(args=args)
    node = STELLA_Arduino_Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # Exception handling to remove unnecessary logs
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception as e:
                pass
        else:
            pass
  
if __name__ == '__main__':
  main()
