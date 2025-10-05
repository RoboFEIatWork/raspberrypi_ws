#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import board
import busio
from adafruit_pca9685 import PCA9685
import math

class PCA9685HardwareInterface(Node):
    def __init__(self):
        super().__init__('pca9685_hardware_interface')
        self.PWM_FREQ = 488
        self.max_robot_linear_speed = 1.5
        self.wheel_radius = 0.05
        self.gear_ratio = 28.0
        self.max_motor_rpm = 8022.0
        self.min_pwm = 0
        self.zero_pwm = 32768
        self.max_pwm = 65535

        self.max_wheel_rad_s_limit = self.max_robot_linear_speed / self.wheel_radius
        max_wheel_rpm_from_motor = self.max_motor_rpm / self.gear_ratio
        max_wheel_rad_s_motor = (max_wheel_rpm_from_motor * 2 * math.pi) / 60.0
        self.max_wheel_rad_s = min(self.max_wheel_rad_s_limit, max_wheel_rad_s_motor)
        pwm_range = min(self.zero_pwm - self.min_pwm, self.max_pwm - self.zero_pwm)
        self.k_pwm = pwm_range / self.max_wheel_rad_s

        # Endereço I2C explícito para evitar confusão
        self.PCA9685_ADDRESS = 0x40  # Endereço padrão do PCA9685
        
        try:
            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.pca = PCA9685(self.i2c, address=self.PCA9685_ADDRESS)
            self.pca.frequency = self.PWM_FREQ
        except Exception as e:
            self.get_logger().error(f'Falha ao inicializar PCA9685: {e}')
            raise

        self.joint_names = [
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'back_left_wheel_joint',
            'back_right_wheel_joint'
        ]
        self.motor_channels = {
            'front_left_wheel_joint': 10,
            'front_right_wheel_joint': 15,
            'back_left_wheel_joint': 11,
            'back_right_wheel_joint': 14
        }

        self.last_pwm_values = [self.zero_pwm] * 4

        # Assina comandos do hardware interface
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/wheel_commands_raw',
            self.wheel_command_callback,
            10
        )
        
        self.get_logger().info('Subscription criada no tópico /wheel_commands_raw')

        for ch in self.motor_channels.values():
            try:
                self.pca.channels[ch].duty_cycle = self.zero_pwm
            except Exception as e:
                self.get_logger().warn(f'Falha ao setar neutro no canal {ch}: {e}')

        self.get_logger().info(
            f'PCA9685 iniciado no endereço I2C 0x{self.PCA9685_ADDRESS:02X}. limite_roda={self.max_wheel_rad_s_limit:.2f} motor={max_wheel_rad_s_motor:.2f} usado={self.max_wheel_rad_s:.2f} k_pwm={self.k_pwm:.2f}'
        )

    def __del__(self):
        try:
            for ch in self.motor_channels.values():
                self.pca.channels[ch].duty_cycle = self.zero_pwm
        except Exception:
            pass

    def wheel_command_callback(self, msg: Float64MultiArray):
        if len(msg.data) != 4:
            self.get_logger().warn('Comando de roda deve ter 4 valores (FL, FR, RL, RR)')
            return
            
        # Log dos comandos recebidos (debug)
        self.get_logger().debug(f'Comandos recebidos: FL={msg.data[0]:.2f}, FR={msg.data[1]:.2f}, RL={msg.data[2]:.2f}, RR={msg.data[3]:.2f}')
        
        # Converte velocidades em PWM
        pwm_values = []
        for idx, vel in enumerate(msg.data):
            joint = self.joint_names[idx]
            # Inverte direção para rodas esquerdas (ajuste conforme sua configuração)
            if joint in ('front_left_wheel_joint', 'back_left_wheel_joint'):
                vel = -vel
            
            pwm = int(self.zero_pwm + self.k_pwm * vel)
            pwm = max(self.min_pwm, min(self.max_pwm, pwm))
            pwm_values.append(pwm)
        
        self.last_pwm_values = pwm_values

        # Aplica PWM aos canais
        for idx, joint in enumerate(self.joint_names):
            ch = self.motor_channels[joint]
            try:
                self.pca.channels[ch].duty_cycle = pwm_values[idx]
            except Exception as e:
                self.get_logger().error(f'Falha ao escrever PWM {pwm_values[idx]} no canal {ch} ({joint}): {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PCA9685HardwareInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
            try:
                node.destroy_node()
            except Exception:
                pass
            # Avoid RCLError: shutdown already called
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass

if __name__ == '__main__':
    main()