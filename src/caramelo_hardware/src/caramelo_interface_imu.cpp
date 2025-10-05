#include "caramelo_hardware/caramelo_interface_imu.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <algorithm>

namespace caramelo_hardware
{
    CarameloInterfaceIMU::CarameloInterfaceIMU()
        : wheel_radius_(0.05), wheel_separation_(0.310), wheel_base_(0.470), imu_data_received_(false)
    {
        // Calcula parâmetro L para equações mecanum
        L_ = (wheel_separation_ + wheel_base_) / 2.0;
        
        // Inicializa twist zerado
        current_twist_.linear.x = 0.0;
        current_twist_.linear.y = 0.0;
        current_twist_.angular.z = 0.0;
    }

    CarameloInterfaceIMU::~CarameloInterfaceIMU()
    {
        // Não há recursos específicos para limpar
    }

    CallbackReturn CarameloInterfaceIMU::on_init(const hardware_interface::HardwareInfo & hardware_info)
    {
        CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
        if(result != CallbackReturn::SUCCESS)
        {
            return result;
        }

        // Inicializa vetores baseado no número de joints definido no URDF
        velocity_commands_.reserve(info_.joints.size());
        positions_states_.reserve(info_.joints.size());
        velocity_states_.reserve(info_.joints.size());
        velocity_commands_.assign(info_.joints.size(), 0.0);
        positions_states_.assign(info_.joints.size(), 0.0);
        velocity_states_.assign(info_.joints.size(), 0.0);
        
        last_imu_time_ = rclcpp::Clock().now();

        // Cria o nó ROS para publicar comandos e subscrever IMU
        node_ = std::make_shared<rclcpp::Node>("caramelo_interface_imu_node");
        
        // Publisher para comandos das rodas
        wheel_cmd_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/wheel_commands_raw", 10);
        
        // Subscriber para dados da IMU
        imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            std::bind(&CarameloInterfaceIMU::imuCallback, this, std::placeholders::_1));

        RCLCPP_INFO(rclcpp::get_logger("CarameloInterfaceIMU"), 
                   "Hardware Interface IMU inicializado. Parâmetros: r=%.3f, L=%.3f", 
                   wheel_radius_, L_);

        return CallbackReturn::SUCCESS;
    }

    void CarameloInterfaceIMU::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Extrai velocidade angular Z da IMU
        current_twist_.angular.z = msg->angular_velocity.z;
        
        // Para robôs com IMU, assumimos que temos odometria ou outro método
        // para velocidades lineares. Por enquanto, usaremos uma estimativa baseada
        // na aceleração linear (integração simples)
        static double prev_vx = 0.0, prev_vy = 0.0;
        auto current_time = rclcpp::Clock().now();
        double dt = (current_time - last_imu_time_).seconds();
        
        if (dt > 0.0 && dt < 0.1) { // Filtro para valores válidos de dt
            // Integração simples da aceleração para velocidade
            // Nota: Esta é uma aproximação. Idealmente você teria odometria
            double ax = msg->linear_acceleration.x;
            double ay = msg->linear_acceleration.y;
            
            current_twist_.linear.x = prev_vx + ax * dt;
            current_twist_.linear.y = prev_vy + ay * dt;
            
            // Aplica filtro passa-baixa para suavizar
            current_twist_.linear.x = 0.8 * prev_vx + 0.2 * current_twist_.linear.x;
            current_twist_.linear.y = 0.8 * prev_vy + 0.2 * current_twist_.linear.y;
            
            prev_vx = current_twist_.linear.x;
            prev_vy = current_twist_.linear.y;
        }
        
        last_imu_time_ = current_time;
        imu_data_received_ = true;
        
        // Calcula velocidades das rodas usando cinemática mecanum
        computeWheelVelocities(current_twist_.linear.x, current_twist_.linear.y, current_twist_.angular.z);
    }

    void CarameloInterfaceIMU::computeWheelVelocities(double vx, double vy, double wz)
    {
        // Verifica se temos exatamente 4 rodas para mecanum
        if(velocity_states_.size() != 4) {
            RCLCPP_WARN_ONCE(rclcpp::get_logger("CarameloInterfaceIMU"), 
                             "Robô mecanum requer exatamente 4 rodas, mas encontradas %zu", 
                             velocity_states_.size());
            return;
        }
        
        // Equações de cinemática direta do mecanum:
        // w1 = (vx - vy - L*wz) / r    # FL (Front Left)
        // w2 = (vx + vy + L*wz) / r    # FR (Front Right)  
        // w3 = (vx + vy - L*wz) / r    # RL (Rear Left)
        // w4 = (vx - vy + L*wz) / r    # RR (Rear Right)
        
        velocity_states_[0] = (vx - vy - L_ * wz) / wheel_radius_;  // FL
        velocity_states_[1] = (vx + vy + L_ * wz) / wheel_radius_;  // FR
        velocity_states_[2] = (vx + vy - L_ * wz) / wheel_radius_;  // RL  
        velocity_states_[3] = (vx - vy + L_ * wz) / wheel_radius_;  // RR
    }

    std::vector<hardware_interface::StateInterface> CarameloInterfaceIMU::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for(size_t i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints.at(i).name,
                hardware_interface::HW_IF_POSITION,
                &positions_states_.at(i)));

            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints.at(i).name,
                hardware_interface::HW_IF_VELOCITY,
                &velocity_states_.at(i)));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> CarameloInterfaceIMU::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for(size_t i = 0; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints.at(i).name,
                hardware_interface::HW_IF_VELOCITY,
                &velocity_commands_.at(i)));
        }

        return command_interfaces;
    }

    CallbackReturn CarameloInterfaceIMU::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("CarameloInterfaceIMU"), "Ativando hardware interface com IMU...");
        
        // Zera todos os estados
        velocity_commands_ = {0.0, 0.0, 0.0, 0.0};
        positions_states_ = {0.0, 0.0, 0.0, 0.0};
        velocity_states_ = {0.0, 0.0, 0.0, 0.0};
        
        // Reinicia estado da IMU
        imu_data_received_ = false;
        current_twist_.linear.x = 0.0;
        current_twist_.linear.y = 0.0;
        current_twist_.angular.z = 0.0;
        
        last_imu_time_ = rclcpp::Clock().now();

        RCLCPP_INFO(rclcpp::get_logger("CarameloInterfaceIMU"), 
                   "Hardware ativado com sucesso. Aguardando dados da IMU em /imu/data");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn CarameloInterfaceIMU::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("CarameloInterfaceIMU"), "Desligando hardware do robo...");
        
        // Para os motores enviando velocidades zero
        std_msgs::msg::Float64MultiArray stop_msg;
        stop_msg.data = {0.0, 0.0, 0.0, 0.0};
        wheel_cmd_pub_->publish(stop_msg);

        RCLCPP_INFO(rclcpp::get_logger("CarameloInterfaceIMU"), "Hardware desativado com sucesso");
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type CarameloInterfaceIMU::read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        // Processa callbacks da IMU
        rclcpp::spin_some(node_);
        
        // Integra posições das rodas usando velocidades calculadas
        double dt = period.seconds();
        if(dt > 0.0 && dt < 0.1) { // Filtro de sanidade
            for(size_t i = 0; i < info_.joints.size(); i++) {
                positions_states_[i] += velocity_states_[i] * dt;
            }
        }
        
        // Log de debug (opcional, pode ser removido)
        static int log_counter = 0;
        if(++log_counter % 100 == 0) { // Log a cada 100 chamadas
            RCLCPP_DEBUG(rclcpp::get_logger("CarameloInterfaceIMU"), 
                        "Vel states: FL=%.2f FR=%.2f RL=%.2f RR=%.2f", 
                        velocity_states_[0], velocity_states_[1], 
                        velocity_states_[2], velocity_states_[3]);
        }
        
        return hardware_interface::return_type::OK;
    }
    
    hardware_interface::return_type CarameloInterfaceIMU::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        // Publica comandos de velocidade para o PCA9685
        std_msgs::msg::Float64MultiArray wheel_cmd_msg;
        wheel_cmd_msg.data = velocity_commands_;  // [fl, fr, rl, rr]
        
        try
        {
            wheel_cmd_pub_->publish(wheel_cmd_msg);
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("CarameloInterfaceIMU"),
                                "Erro ao publicar comandos de velocidade: " << e.what());
            return hardware_interface::return_type::ERROR;
        }
        
        return hardware_interface::return_type::OK;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(caramelo_hardware::CarameloInterfaceIMU, hardware_interface::SystemInterface)