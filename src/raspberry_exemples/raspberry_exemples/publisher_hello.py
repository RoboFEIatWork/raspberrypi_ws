#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime, timezone

class HelloPublisher(Node):
    def __init__(self):
        super().__init__('hello_publisher')
        self.pub = self.create_publisher(String, '/raspberry_helloword', 10)
        self.timer = self.create_timer(0.5, self.timer_cb)  # 2 msgs/seg

    def timer_cb(self):
        # timestamp ISO-8601 em UTC
        ts = datetime.now(timezone.utc).isoformat(timespec='seconds')
        msg = String()
        msg.data = f'hello word @ {ts}'
        self.pub.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main():
    rclpy.init()
    node = HelloPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
