#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class LaneControlNode : public rclcpp::Node
{
public:
  LaneControlNode() : Node("lane_control_node")
  {
    this->declare_parameter("speed",     0.15);
    this->declare_parameter("k",         1.0);
    this->declare_parameter("max_steer", 0.52);
    this->declare_parameter("timeout",   0.5);

    speed_     = this->get_parameter("speed").as_double();
    k_         = this->get_parameter("k").as_double();
    max_steer_ = this->get_parameter("max_steer").as_double();
    timeout_   = this->get_parameter("timeout").as_double();

    sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "/status_err", 10,
      std::bind(&LaneControlNode::err_callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // 20 Hz control loop
    timer_ = this->create_wall_timer(
      50ms, std::bind(&LaneControlNode::control_loop, this));

    RCLCPP_INFO(this->get_logger(),
      "Stanley lane control: speed=%.2f m/s  k=%.2f  "
      "max_steer=%.1f deg  timeout=%.1f s",
      speed_, k_, max_steer_ * 180.0 / M_PI, timeout_);
  }

private:
  // ── Stanley controller ───────────────────────────────────────────────────
  //   delta     = e_psi + atan(k * e_y / v)
  //   angular_z = v * tan(delta) / L

  static constexpr double WHEELBASE_M = 0.21;

  void err_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    e_y_           = msg->x;
    e_psi_         = msg->y;
    last_err_time_ = this->now();
    has_received_  = true;
  }

  void control_loop()
  {
    geometry_msgs::msg::Twist twist;  // zero-initialised → stop by default

    if (!has_received_) {
      pub_->publish(twist);
      return;
    }

    double elapsed = (this->now() - last_err_time_).seconds();
    if (elapsed > timeout_) {
      pub_->publish(twist);  // safety stop
      return;
    }

    double v     = speed_;
    double delta = e_psi_ + std::atan2(k_ * e_y_, std::max(v, 0.1));
    delta        = std::clamp(delta, -max_steer_, max_steer_);

    twist.linear.x  = v;
    twist.angular.z = v * std::tan(delta) / WHEELBASE_M;
    pub_->publish(twist);
  }

  // ── Parameters ───────────────────────────────────────────────────────────
  double speed_{0.15};
  double k_{1.0};
  double max_steer_{0.52};
  double timeout_{0.5};

  // ── State ────────────────────────────────────────────────────────────────
  double       e_y_{0.0};
  double       e_psi_{0.0};
  rclcpp::Time last_err_time_;
  bool         has_received_{false};

  // ── ROS interfaces ───────────────────────────────────────────────────────
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr      pub_;
  rclcpp::TimerBase::SharedPtr                                  timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaneControlNode>());
  rclcpp::shutdown();
  return 0;
}
