#include "caramelo_hardware/caramelo_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <sstream>
#include <iomanip>

namespace caramelo_hardware
{
    CarameloInterface::CarameloInterface()
    {

    }

    CarameloInterface::~CarameloInterface()
    {
        if(arduino_.IsOpen())
        {
            try
            {
                arduino_.Close();
            }
            catch(...)
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("CarameloInterface"), "Algo de errado aconteceu quando tentando fechar a comunicação com a porta" << port_);
            }
            
        }
    }

    CallbackReturn CarameloInterface::on_init(const hardware_interface::HardwareInfo & hardware_info)
    {
        CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
        if(result != CallbackReturn::SUCCESS)
        {
            return result;
        }

        try
        {
            port_ = info_.hardware_parameters.at("port");
        }
        catch(const std::out_of_range &e)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("CarameloInterface"), "Porta não especificada");
            return CallbackReturn::FAILURE;
        }

        velocity_commands_.reserve(info_.joints.size());
        positions_states_.reserve(info_.joints.size());
        velocity_states_.reserve(info_.joints.size());
        last_run_ = rclcpp::Clock().now();

        // Cria o nó ROS para publicar comandos
        node_ = std::make_shared<rclcpp::Node>("caramelo_interface_node");
        wheel_cmd_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/wheel_commands_raw", 10);

        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> CarameloInterface::export_state_interfaces()
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

    std::vector<hardware_interface::CommandInterface> CarameloInterface::export_command_interfaces()
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

    CallbackReturn CarameloInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("CarameloInterface"), "Ativando hardware interface...");
        
        // Zera todos os estados
        velocity_commands_ = {0.0, 0.0, 0.0, 0.0};
        positions_states_ = {0.0, 0.0, 0.0, 0.0};
        velocity_states_ = {0.0, 0.0, 0.0, 0.0};

        try
        {
            arduino_.Open(port_);
            arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
            arduino_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
            arduino_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
            arduino_.SetParity(LibSerial::Parity::PARITY_NONE);
            arduino_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
        }
        catch(...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("CarameloInterface"), "Não foi possível abrir a porta " << port_);
            return CallbackReturn::FAILURE;
        }

        RCLCPP_INFO(rclcpp::get_logger("CarameloInterface"), "Hardware ativado com sucesso. Arduino conectado e publisher criado.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn CarameloInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("CarameloInterface"), "Desligando hardware do robo...");
        
        // Para os motores enviando velocidades zero
        std_msgs::msg::Float64MultiArray stop_msg;
        stop_msg.data = {0.0, 0.0, 0.0, 0.0};
        wheel_cmd_pub_->publish(stop_msg);
        
        if(arduino_.IsOpen())
        {
            try
            {
                arduino_.Close();
            }
            catch(...)
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("CarameloInterface"), "Erro ao fechar comunicação com a porta " << port_);
                return CallbackReturn::FAILURE;
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("CarameloInterface"), "Hardware desativado com sucesso");
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type CarameloInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        // Lê dados dos 4 encoders do Arduino
        if(arduino_.IsDataAvailable())
        {
            auto dt = (rclcpp::Clock().now() - last_run_).seconds();
            std::string message;
            arduino_.ReadLine(message);
            
            // Remove caracteres de quebra de linha
            message.erase(std::remove(message.begin(), message.end(), '\r'), message.end());
            message.erase(std::remove(message.begin(), message.end(), '\n'), message.end());
            
            std::stringstream ss(message);
            std::string encoder_value;
            int encoder_idx = 0;
            
            // Parse formato: fl_value,fr_value,rl_value,rr_value
            while(std::getline(ss, encoder_value, ',') && encoder_idx < 4)
            {
                try 
                {
                    double current_velocity = std::stod(encoder_value);
                    velocity_states_[encoder_idx] = current_velocity;
                    // Integra posição: pos += vel * dt
                    positions_states_[encoder_idx] += current_velocity * dt;
                    encoder_idx++;
                }
                catch(const std::exception& e)
                {
                    RCLCPP_WARN(rclcpp::get_logger("CarameloInterface"), 
                               "Erro ao parsear encoder %d: %s", encoder_idx, e.what());
                    break;
                }
            }
            
            if(encoder_idx == 4) 
            {
                last_run_ = rclcpp::Clock().now();
            }
        }
        return hardware_interface::return_type::OK;
    }
    
    hardware_interface::return_type CarameloInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        // Publica comandos de velocidade para o PCA9685 (não escreve no Arduino)
        std_msgs::msg::Float64MultiArray wheel_cmd_msg;
        wheel_cmd_msg.data = velocity_commands_;  // [fl, fr, rl, rr]
        
        try
        {
            wheel_cmd_pub_->publish(wheel_cmd_msg);
            
            // Processa callbacks do ROS para manter o publisher funcionando
            rclcpp::spin_some(node_);
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("CarameloInterface"),
                                "Erro ao publicar comandos de velocidade: " << e.what());
            return hardware_interface::return_type::ERROR;
        }
        
        return hardware_interface::return_type::OK;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(caramelo_hardware::CarameloInterface, hardware_interface::SystemInterface)