#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import board
import busio
from adafruit_pca9685 import PCA9685

class PCA9685Controller(Node):
    def __init__(self):
        super().__init__('pca9685_controller')
        self.get_logger().info('PCA9685 Controller started')

        # Initialize I2C and PCA9685
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 1000  # Set PWM frequency

        # Subscription to wheel commands
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'wheel_commands_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # PCA9685 has 16 channels
        self.motor_channels = [0, 1, 2, 3] # Example: channels for 4 motors

    def listener_callback(self, msg):
        # msg.data contains wheel speeds, e.g., [-1.0 to 1.0]
        # This logic needs to be adapted to convert wheel speeds to PWM duty cycles
        # For example, mapping [-1.0, 1.0] to [0, 0xFFFF]
        if len(msg.data) >= len(self.motor_channels):
            for i, channel_index in enumerate(self.motor_channels):
                speed = msg.data[i]
                # Basic conversion - THIS IS A PLACEHOLDER
                # You need to implement the correct logic for your motors
                if speed > 0: # Forward
                    duty_cycle = int(speed * 0xFFFF)
                elif speed < 0: # Backward
                    duty_cycle = int(-speed * 0xFFFF)
                else: # Stop
                    duty_cycle = 0
                
                # This assumes a simple setup. Real motor drivers might need
                # separate pins for direction and speed.
                self.pca.channels[channel_index].duty_cycle = duty_cycle
                # self.get_logger().info(f'Motor {i} (Channel {channel_index}): Speed {speed:.2f}, Duty Cycle {duty_cycle}')
        else:
            self.get_logger().warn('Received wheel_commands_raw with insufficient data.')

def main(args=None):
    rclpy.init(args=args)
    pca_controller = PCA9685Controller()
    rclpy.spin(pca_controller)
    pca_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
