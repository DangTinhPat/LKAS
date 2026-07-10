#include <chrono>
#include <memory>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "mcu_agent/agent_supervisor.hpp"

using namespace std::chrono_literals;

// Owns the serial connection to the ESP32 for the whole workspace: only one process may hold
// the port, so this node is the single supervisor of the micro_ros_agent process. Once running,
// the ESP32's micro-ROS publishers/subscribers (see README.md for the topic contract) appear as
// ordinary ROS 2 topics — no further translation is needed here.
class McuAgentNode : public rclcpp::Node
{
public:
  McuAgentNode() : Node("mcu_agent_node")
  {
    this->declare_parameter("serial_port", "/dev/ttyACM0");
    this->declare_parameter("baud_rate", 115200);
    this->declare_parameter("agent_executable", "micro_ros_agent");
    this->declare_parameter("restart_delay_sec", 2.0);

    mcu_agent::AgentSupervisor::Options opts;
    opts.serial_port = this->get_parameter("serial_port").as_string();
    opts.baud_rate = static_cast<int>(this->get_parameter("baud_rate").as_int());
    opts.agent_executable = this->get_parameter("agent_executable").as_string();
    restart_delay_sec_ = this->get_parameter("restart_delay_sec").as_double();

    supervisor_ = std::make_unique<mcu_agent::AgentSupervisor>(opts);
    status_pub_ = this->create_publisher<std_msgs::msg::Bool>("/mcu/agent/status", 10);

    RCLCPP_INFO(
      this->get_logger(), "[mcu_agent] starting micro_ros_agent on %s @ %d baud",
      opts.serial_port.c_str(), opts.baud_rate);
    supervisor_->start();

    watchdog_timer_ = this->create_wall_timer(1s, std::bind(&McuAgentNode::tick, this));
  }

  ~McuAgentNode() override { supervisor_->stop(); }

private:
  void tick()
  {
    bool running = supervisor_->poll_running();
    if (!running) {
      const auto now = this->now();
      const bool cooled_down =
        !last_restart_attempt_ || (now - *last_restart_attempt_).seconds() >= restart_delay_sec_;
      if (cooled_down) {
        RCLCPP_WARN(this->get_logger(), "[mcu_agent] micro_ros_agent not running — (re)starting");
        supervisor_->start();
        last_restart_attempt_ = now;
        running = supervisor_->poll_running();
      }
    }

    std_msgs::msg::Bool msg;
    msg.data = running;
    status_pub_->publish(msg);
  }

  std::unique_ptr<mcu_agent::AgentSupervisor> supervisor_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  double restart_delay_sec_{2.0};
  std::optional<rclcpp::Time> last_restart_attempt_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<McuAgentNode>());
  rclcpp::shutdown();
  return 0;
}
