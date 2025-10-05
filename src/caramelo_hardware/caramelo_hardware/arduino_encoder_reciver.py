#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String

import serial

class ArduinoEncoderReceiver(Node):
    def __init__(self):
        super().__init__('arduino_encoder_receiver')

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)

        self.port_ = self.get_parameter("port").value
        self.baudrate_ = self.get_parameter("baudrate").value

        self.pub_ = self.create_publisher(String, "serial_receiver", 10)
        self.arduino_ = serial.Serial(port=self.port_, baudrate=self.baudrate_, timeout=0.1)

        self.frequency_ = 0.01
        self.timer_ = self.create_timer(self.frequency_, self.timerCallback)

        self.get_logger().info('Inicializando leitura dos encoders...')
        self.get_logger().info('Publicando em uma frquência de %d Hz' % (1/self.frequency_))

    def timerCallback(self):
        if rclpy.ok() and self.arduino_.is_open:
            data = self.arduino_.readline()

            try:
                data.decode("utf-8")
            except:
                return

            msg = String()
            msg.data = str(data)
            self.pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    serial_reciver = ArduinoEncoderReceiver()
    try:
        rclpy.spin(serial_reciver)
    except KeyboardInterrupt:
        pass
    serial_reciver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()