#pragma once

#include <map>
#include <mutex>
#include <string>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace main_bot_hardware
{

// ros2_control SystemInterface for the real robot. Mirrors gz_ros2_control's role in
// simulation/ros2_control.xacro: the exact same joint layout, but read()/write() move data
// through /mcu/joint_states and /mcu/joint_commands instead of the physics engine. Those two
// topics are bridged from the ESP32 by the micro-ROS agent (see the mcu_agent package) — this
// class only talks ROS 2, never touches serial/XRCE-DDS directly.
class RealRobotSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_commands_pub_;

  // Latest values reported by the MCU, keyed by joint name. front_left_wheel_joint and
  // front_right_wheel_joint are passive/unsensed on the real robot (no encoder) — they are
  // never written by the MCU and stay at their initial 0.0, matching the note in
  // description/practical/ros2_control.xacro.
  std::mutex state_mutex_;
  std::map<std::string, double> latest_position_;
  std::map<std::string, double> latest_velocity_;
};

}  // namespace main_bot_hardware
