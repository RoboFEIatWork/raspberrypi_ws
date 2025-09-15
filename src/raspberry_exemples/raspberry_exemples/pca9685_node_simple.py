import board
import busio
from adafruit_pca9685 import PCA9685
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class PCA9685Node(Node):
    """Node para controlar 4 motores via PCA9685 usando valores crus (raw) 16-bit.

    Publicar diretamente o duty_cycle desejado (0..65535) em cada tópico:
      /motor_fr, /motor_br, /motor_fl, /motor_bl  (std_msgs/Int32)

    Convenção sugerida (se aplicável ao seu ESC):
      0      = trás máximo
      32767  ≈ parado (50%)
      65535  = frente máximo

    Não há inversão automática aqui. O valor enviado é aplicado direto ao canal.
    """

    def __init__(self):
        super().__init__('pca9685_motors')

        self.FREQ_HZ = 490
        self.OSC_HZ = 25_276_826

        # Inicializa PCA9685
        try:
            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.pca = PCA9685(self.i2c)
        except Exception as e:  # hardware error
            self.get_logger().error(f'Erro inicializando I2C/PCA9685: {e}')
            raise

        self.pca.reference_clock_speed = self.OSC_HZ
        self.pca.frequency = self.FREQ_HZ

        self.motor_channels = {
            'FR': self.pca.channels[15],  # Front Right
            'BR': self.pca.channels[14],  # Back Right
            'FL': self.pca.channels[10],  # Front Left
            'BL': self.pca.channels[11],  # Back Left
        }

        # Valor neutro inicial (50%)
        self.neutral = 32767
        for ch in self.motor_channels.values():
            ch.duty_cycle = self.neutral
        self.get_logger().info('Motores em neutro (32767). Pronto para receber comandos raw 0..65535.')

        # Subscribers para cada motor
        self.sub_fr = self.create_subscription(Int32, 'motor_fr', lambda m: self.set_raw('FR', m.data), 10)
        self.sub_br = self.create_subscription(Int32, 'motor_br', lambda m: self.set_raw('BR', m.data), 10)
        self.sub_fl = self.create_subscription(Int32, 'motor_fl', lambda m: self.set_raw('FL', m.data), 10)
        self.sub_bl = self.create_subscription(Int32, 'motor_bl', lambda m: self.set_raw('BL', m.data), 10)

    def set_raw(self, name: str, value: int):
        if name not in self.motor_channels:
            self.get_logger().warning(f'Motor desconhecido: {name}')
            return
        # Clamp 0..65535
        if value < 0:
            value = 0
        elif value > 65535:
            value = 65535
        self.motor_channels[name].duty_cycle = value
        self.get_logger().debug(f'Motor {name} duty={value}')

    def destroy_node(self):
        self.get_logger().info('Desligando motores (duty=0).')
        for ch in self.motor_channels.values():
            ch.duty_cycle = 0
        try:
            self.pca.deinit()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PCA9685Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
