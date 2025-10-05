#ifndef CARAMELO_INTERFACE_IMU_HPP
#define CARAMELO_INTERFACE_IMU_HPP

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <vector>
#include <string>
#include <memory>
#include <cmath>

namespace caramelo_hardware
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class CarameloInterfaceIMU : public hardware_interface::SystemInterface
{
public:
    CarameloInterfaceIMU();
    virtual ~CarameloInterfaceIMU();

    virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    virtual CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;

    virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    virtual hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    virtual hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
    // Nó ROS e comunicação
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    
    // Estados das juntas
    std::vector<double> velocity_commands_;     // 4 comandos de velocidade das rodas
    std::vector<double> positions_states_;      // 4 posições das rodas (integradas)
    std::vector<double> velocity_states_;       // 4 velocidades das rodas calculadas
    
    // Dados da IMU
    geometry_msgs::msg::Twist current_twist_;   // Twist atual do robô
    rclcpp::Time last_imu_time_;
    bool imu_data_received_;
    
    // Parâmetros físicos do robô mecanum (em metros)
    double wheel_radius_;       // 0.05m
    double wheel_separation_;   // 0.310m (distância entre rodas esquerda/direita)  
    double wheel_base_;         // 0.470m (distância entre rodas frente/trás)
    double L_;                  // Parâmetro L = (wheel_separation + wheel_base) / 2
    
    // Callback da IMU
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    
    // Cinemática mecanum: converte twist do robô para velocidades das rodas
    void computeWheelVelocities(double vx, double vy, double wz);


};
}

#endif // CARAMELO_INTERFACE_IMU_HPP