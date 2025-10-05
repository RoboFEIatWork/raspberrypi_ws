#ifndef CARAMELO_INTERFACE_HPP
#define CARAMELO_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <libserial/SerialPort.h>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <vector>
#include <string>
#include <memory>

namespace caramelo_hardware
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class CarameloInterface : public hardware_interface::SystemInterface
{
public:
    CarameloInterface();
    virtual ~CarameloInterface();

    virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    virtual CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;

    virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    virtual hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    virtual hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
    LibSerial::SerialPort arduino_;
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_pub_;

    std::string port_;
    std::vector<double> velocity_commands_;     // 4 comandos de velocidade
    std::vector<double> positions_states_;      // 4 posições dos encoders
    std::vector<double> velocity_states_;       // 4 velocidades dos encoders

    rclcpp::Time last_run_;


};
}

#endif // CARAMELO_INTERFACE_HPP