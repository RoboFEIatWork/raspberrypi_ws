#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import board
import busio
from adafruit_pca9685 import PCA9685
import time
import math

class PCA9685HardwareInterface(Node):
    def __init__(self):
        super().__init__('pca9685_hardware_interface')
        # Frequência e limites (16-bit duty_cycle)
        self.PWM_FREQ = 488
        self.min_pwm = 0
        self.zero_pwm = 32768   # neutro
        self.max_pwm = 65535

        # Parâmetros físicos
        self.max_motor_rpm = 8022.0
        self.gear_ratio = 28.0
        self.wheel_radius = 0.05  # m

        # Ganho PWM (rad/s -> duty variation)
        max_wheel_rpm = self.max_motor_rpm / self.gear_ratio
        max_wheel_rad_s = (max_wheel_rpm * 2 * math.pi) / 60.0
        pwm_range = min(self.zero_pwm - self.min_pwm, self.max_pwm - self.zero_pwm)
        self.k_pwm = pwm_range / max_wheel_rad_s

        # I2C + PCA9685
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = self.PWM_FREQ

        # Mapeamento canais
        # Ordem esperada na mensagem: [FL, FR, RL, RR]
        self.joint_names = [
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'back_left_wheel_joint',
            'back_right_wheel_joint'
        ]
        self.motor_channels = {
            'front_left_wheel_joint': 0,
            'front_right_wheel_joint': 1,
            'back_left_wheel_joint': 2,
            'back_right_wheel_joint': 3
        }

        # Estados simulados
        self.sim_position = [0.0] * 4
        self.sim_velocity = [0.0] * 4
        self.last_time = self.get_clock().now()

        # Armazenamento para logs/estado
        self.last_wheel_velocities = [0.0] * 4
        self.last_pwm_values = [self.zero_pwm] * 4
        self.last_cmd_vel = {'linear_x': 0.0, 'linear_y': 0.0, 'angular_z': 0.0}

        # Publishers / Subscribers
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/mecanum_controller/commands',
            self.wheel_command_callback,
            10
        )
        # Timer para republicar estados mesmo sem novos comandos
        self.timer = self.create_timer(0.1, self.publish_joint_states)

        # Inicializa motores em neutro
        for ch in self.motor_channels.values():
            self.pca.channels[ch].duty_cycle = self.zero_pwm

        self.get_logger().info('PCA9685 node inicializado.')

    def __del__(self):
        try:
            for ch in self.motor_channels.values():
                self.pca.channels[ch].duty_cycle = self.zero_pwm
        except:
            pass

    def wheel_command_callback(self, msg: Float64MultiArray):
        if len(msg.data) != 4:
            self.get_logger().warn('Comando de roda deve ter 4 valores (FL, FR, RL, RR)')
            return

        self.last_wheel_velocities = list(msg.data)

        # Aproximações de cmd_vel para log
        v_fl, v_fr, v_rl, v_rr = msg.data
        wheel_radius = self.wheel_radius
        v_x_approx = (v_fl + v_fr + v_rl + v_rr) * wheel_radius / 4.0
        v_y_approx = (-v_fl + v_fr + v_rl - v_rr) * wheel_radius / 4.0
        w_z_approx = (-v_fl + v_fr - v_rl + v_rr) * wheel_radius / (4 * 0.35)
        self.last_cmd_vel = {
            'linear_x': v_x_approx,
            'linear_y': v_y_approx,
            'angular_z': w_z_approx
        }

        # Cálculo PWM (direção frente = menor que neutro)
        velocities = [v_fl, v_fr, v_rl, v_rr]
        pwm_values = []
        for vel in velocities:
            pwm = int(self.zero_pwm - self.k_pwm * vel)
            pwm = max(self.min_pwm, min(self.max_pwm, pwm))
            pwm_values.append(pwm)

        self.last_pwm_values = pwm_values

        # Aplicar aos canais (ordem consistente)
        for idx, joint in enumerate(self.joint_names):
            ch = self.motor_channels[joint]
            try:
                self.pca.channels[ch].duty_cycle = pwm_values[idx]
            except Exception as e:
                self.get_logger().error(f'Falha ao escrever PWM no canal {ch}: {e}')

        # Atualizar simulação de estado
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt > 0:
            for i in range(4):
                self.sim_velocity[i] = velocities[i]
                self.sim_position[i] += self.sim_velocity[i] * dt
        self.last_time = now

        # Publicar imediatamente
        self.publish_joint_states()

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.sim_position
        msg.velocity = self.sim_velocity
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PCA9685HardwareInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()