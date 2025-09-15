#include "caramelo_hardware/proxy_system.hpp"
#include <algorithm>
#include <cmath>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace caramelo_hardware {

hardware_interface::CallbackReturn CarameloProxySystem::on_init(const hardware_interface::HardwareInfo & info) {
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_names_.clear();
  for (const auto & joint : info.joints) {
    joint_names_.push_back(joint.name);
    if (joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_ERROR(rclcpp::get_logger("CarameloProxySystem"), "Cada junta deve ter exatamente 1 interface de comando velocity");
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() < 2) {
      RCLCPP_ERROR(rclcpp::get_logger("CarameloProxySystem"), "Cada junta deve ter pelo menos position e velocity como state interfaces");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  pos_.assign(joint_names_.size(), 0.0);
  vel_cmd_.assign(joint_names_.size(), 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> CarameloProxySystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> states;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    states.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_[i]);
    // We reuse vel_cmd_ as reported velocity (since we don't have feedback yet)
    states.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_cmd_[i]);
  }
  return states;
}

std::vector<hardware_interface::CommandInterface> CarameloProxySystem::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> cmds;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    cmds.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_cmd_[i]);
  }
  return cmds;
}

hardware_interface::CallbackReturn CarameloProxySystem::on_configure(const rclcpp_lifecycle::State &) {
  node_ = std::make_shared<rclcpp::Node>("caramelo_hardware_proxy_node");
  wheel_cmd_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/wheel_commands_raw", 10);
  RCLCPP_INFO(node_->get_logger(), "CarameloProxySystem configurado. Publicando comandos em /wheel_commands_raw");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CarameloProxySystem::on_activate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(node_->get_logger(), "CarameloProxySystem ativado");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CarameloProxySystem::on_deactivate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(node_->get_logger(), "CarameloProxySystem desativado");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type CarameloProxySystem::read(const rclcpp::Time &, const rclcpp::Duration & period) {
  // Simples integração de posição: pos += vel * dt (sem feedback real)
  double dt = period.seconds();
  for (size_t i = 0; i < pos_.size(); ++i) {
    pos_[i] += vel_cmd_[i] * dt;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CarameloProxySystem::write(const rclcpp::Time &, const rclcpp::Duration &) {
  // Envia comandos atuais
  std_msgs::msg::Float64MultiArray msg;
  msg.data = vel_cmd_;
  wheel_cmd_pub_->publish(msg);
  return hardware_interface::return_type::OK;
}

} // namespace caramelo_hardware

PLUGINLIB_EXPORT_CLASS(caramelo_hardware::CarameloProxySystem, hardware_interface::SystemInterface)
