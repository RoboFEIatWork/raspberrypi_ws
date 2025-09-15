"""Node ROS2 super simples que publica contagens brutas de 4 encoders.

Estilo inspirado no exemplo ESP32 fornecido: ler contadores e publicar em taxa fixa.
Publica Int32MultiArray em /encoders na ordem: front_left, front_right, back_left, back_right.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from .encoder_simple import SimpleEncoder

# Definição fixa de pinos (BCM) conforme informado
FL_A, FL_B = 20, 21
FR_A, FR_B = 6, 5
BL_A, BL_B = 17, 27
BR_A, BR_B = 26, 16

PUBLISH_HZ = 50  # 20 ms ~ igual ao exemplo (sendInterval = 20ms)


class EncoderNode(Node):
    def __init__(self):
        super().__init__('encoder_publisher')
        self.get_logger().info('Inicializando encoders...')
        self.enc_fl = SimpleEncoder(FL_A, FL_B)
        self.enc_fr = SimpleEncoder(FR_A, FR_B)
        self.enc_bl = SimpleEncoder(BL_A, BL_B)
        self.enc_br = SimpleEncoder(BR_A, BR_B)
        self.pub = self.create_publisher(Int32MultiArray, 'encoders', 10)
        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self._tick)
        self._log_counter = 0
        self.get_logger().info('Encoders prontos.')

    def _tick(self):
        msg = Int32MultiArray()
        msg.data = [
            self.enc_fl.get_count(),
            self.enc_fr.get_count(),
            self.enc_bl.get_count(),
            self.enc_br.get_count(),
        ]
        self.pub.publish(msg)
        # Log a cada segundo
        self._log_counter += 1
        if self._log_counter >= PUBLISH_HZ:
            self._log_counter = 0
            self.get_logger().info(f"Encoders {msg.data}")

    def destroy_node(self):
        for enc in (self.enc_fl, self.enc_fr, self.enc_bl, self.enc_br):
            try:
                enc.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()