#include "main_bot_hardware/real_robot_system.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace main_bot_hardware
{

hardware_interface::CallbackReturn RealRobotSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto & joint : info_.joints) {
    latest_position_[joint.name] = 0.0;
    latest_velocity_[joint.name] = 0.0;
  }

  node_ = std::make_shared<rclcpp::Node>("main_bot_hardware_bridge");

  joint_states_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/mcu/joint_states", rclcpp::SensorDataQoS(),
    std::bind(&RealRobotSystem::joint_states_callback, this, std::placeholders::_1));

  joint_commands_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
    "/mcu/joint_commands", rclcpp::SystemDefaultsQoS());

  if (auto executor = params.executor.lock()) {
    executor->add_node(node_);
  } else {
    RCLCPP_WARN(
      node_->get_logger(),
      "No controller_manager executor available at on_init — /mcu/joint_states will not be "
      "received until this node is added to an executor.");
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

void RealRobotSystem::joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (i < msg->position.size()) {
      latest_position_[msg->name[i]] = msg->position[i];
    }
    if (i < msg->velocity.size()) {
      latest_velocity_[msg->name[i]] = msg->velocity[i];
    }
  }
}

hardware_interface::return_type RealRobotSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (const auto & joint : info_.joints) {
    const std::string pos_if = joint.name + "/" + hardware_interface::HW_IF_POSITION;
    const std::string vel_if = joint.name + "/" + hardware_interface::HW_IF_VELOCITY;
    if (has_state(pos_if)) {
      set_state(pos_if, latest_position_[joint.name]);
    }
    if (has_state(vel_if)) {
      set_state(vel_if, latest_velocity_[joint.name]);
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RealRobotSystem::write(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  sensor_msgs::msg::JointState cmd_msg;
  cmd_msg.header.stamp = time;

  for (const auto & joint : info_.joints) {
    const std::string pos_if = joint.name + "/" + hardware_interface::HW_IF_POSITION;
    const std::string vel_if = joint.name + "/" + hardware_interface::HW_IF_VELOCITY;
    const bool has_pos_cmd = has_command(pos_if);
    const bool has_vel_cmd = has_command(vel_if);
    if (!has_pos_cmd && !has_vel_cmd) {
      continue;  // passive front wheels: state-only, nothing to command
    }

    cmd_msg.name.push_back(joint.name);
    cmd_msg.position.push_back(has_pos_cmd ? get_command<double>(pos_if) : 0.0);
    cmd_msg.velocity.push_back(has_vel_cmd ? get_command<double>(vel_if) : 0.0);
  }

  joint_commands_pub_->publish(cmd_msg);
  return hardware_interface::return_type::OK;
}

}  // namespace main_bot_hardware

PLUGINLIB_EXPORT_CLASS(main_bot_hardware::RealRobotSystem, hardware_interface::SystemInterface)
