import rclpy
from rclpy.node import Node

from raspberry_bringup.encoder import QuadratureEncoder


class EncoderTestNode(Node):
    def __init__(self) -> None:
        super().__init__('encoder_test_node')
        # Parâmetros ROS (padrões: D20/D21, CPR=4096, invert=False)
        self.declare_parameter('channel_a', 20)
        self.declare_parameter('channel_b', 21)
        self.declare_parameter('cpr', 4096)
        self.declare_parameter('invert', False)

        a = int(self.get_parameter('channel_a').get_parameter_value().integer_value)
        b = int(self.get_parameter('channel_b').get_parameter_value().integer_value)
        cpr = int(self.get_parameter('cpr').get_parameter_value().integer_value)
        invert = bool(self.get_parameter('invert').get_parameter_value().bool_value)

        self.get_logger().info(
            f'Iniciando encoder nos pinos D{a}/D{b}, CPR={cpr}, invert={invert}'
        )
        self.encoder = QuadratureEncoder(a, b, cpr, invert)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self) -> None:
        count = self.encoder.get_count()
        pa, pb = self.encoder.read_pins()
        self.get_logger().info(f'Encoder count: {count} | pins A,B=({pa},{pb})')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EncoderTestNode()
    try:
        rclpy.spin(node)
    finally:
        node.encoder.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()