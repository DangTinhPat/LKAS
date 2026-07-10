#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

// Stanley controller with feed-forward curvature and blended odometry/vision
// curvature estimate.
//
//  /odometry/filtered ──> kappa_odom = vyaw / vx   [50 Hz, from EKF]
//  /status_err        ──> kappa_vis                [10 Hz, from vision, look-ahead]
//                              │
//        kappa_use = kappa_blend*kappa_odom + (1-kappa_blend)*kappa_vis
//              (falls back to kappa_vis when vx < kappa_odom_min_vx)
//                              │
//  kappa_use ──> delta_ff = k_ff * atan(L * kappa_use)
//  e_psi     ──> [dead-band] ──> e_psi_corr = e_psi - lookahead_x*kappa_use
//  e_y       ──> [dead-band] ──> atan2(k*e_y, v_soft)
//                                        │
//                  delta = delta_ff + e_psi_corr + atan2(k*e_y, v_soft)
//                                        │  clamp +-max_steer
//                            raw_az = v*tan(delta) / L
//                                        │
//                 [asymmetric EMA: alpha_normal rising, alpha_conv decaying]
//                                        │
//                               angular_z ──> /cmd_vel
//
// kappa_odom is smooth (EKF gyro/encoder) but reflects the current turn only;
// kappa_vis is predictive (looks ~0.9m ahead) but noisier. Blending the two
// (default 60% odom / 40% vision) balances stability against responsiveness.

class LaneControlNode : public rclcpp::Node
{
public:
  LaneControlNode() : Node("lane_control_node")
  {
    this->declare_parameter("speed",        8.0);
    this->declare_parameter("k",            0.30);
    this->declare_parameter("max_steer",    0.52);
    this->declare_parameter("timeout",      0.5);
    this->declare_parameter("v_softening",  0.20);
    this->declare_parameter("alpha",        0.40);
    this->declare_parameter("alpha_conv",   0.70);
    this->declare_parameter("db_ey",        0.020);
    this->declare_parameter("db_psi",       0.017);
    this->declare_parameter("k_ff",         1.00);
    this->declare_parameter("lookahead_x",  0.923);
    this->declare_parameter("kappa_blend",       0.60);
    this->declare_parameter("kappa_odom_min_vx", 0.05);

    speed_        = this->get_parameter("speed").as_double();
    k_            = this->get_parameter("k").as_double();
    max_steer_    = this->get_parameter("max_steer").as_double();
    timeout_      = this->get_parameter("timeout").as_double();
    v_softening_  = this->get_parameter("v_softening").as_double();
    alpha_        = this->get_parameter("alpha").as_double();
    alpha_conv_   = this->get_parameter("alpha_conv").as_double();
    db_ey_        = this->get_parameter("db_ey").as_double();
    db_psi_       = this->get_parameter("db_psi").as_double();
    k_ff_             = this->get_parameter("k_ff").as_double();
    lookahead_x_      = this->get_parameter("lookahead_x").as_double();
    kappa_blend_      = this->get_parameter("kappa_blend").as_double();
    kappa_odom_min_vx_ = this->get_parameter("kappa_odom_min_vx").as_double();

    sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "/status_err", 10,
      std::bind(&LaneControlNode::err_callback, this, std::placeholders::_1));

    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", 10,
      std::bind(&LaneControlNode::odom_callback, this, std::placeholders::_1));

    sub_offset_ = this->create_subscription<std_msgs::msg::Float64>(
      "/overtake/target_offset", 10,
      [this](std_msgs::msg::Float64::SharedPtr msg) { target_offset_ = msg->data; });

    sub_speed_ = this->create_subscription<std_msgs::msg::Float64>(
      "/overtake/target_speed", 10,
      [this](std_msgs::msg::Float64::SharedPtr msg) {
          if (msg->data > 0.0) speed_ = msg->data;
      });

    sub_state_ = this->create_subscription<std_msgs::msg::String>(
      "/overtake/state", 10,
      [this](std_msgs::msg::String::SharedPtr msg) { overtake_state_ = msg->data; });

    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = this->create_wall_timer(
      50ms, std::bind(&LaneControlNode::control_loop, this));

    RCLCPP_INFO(this->get_logger(),
      "Stanley+FF+κ-blend: speed=%.2f  k=%.2f  v_soft=%.2f  "
      "alpha=%.2f/%.2f  db_ey=%.3f  db_psi=%.1fdeg  "
      "k_ff=%.2f  lookahead_x=%.3fm  kappa_blend=%.2f(odom)/%.2f(vis)",
      speed_, k_, v_softening_,
      alpha_, alpha_conv_, db_ey_, db_psi_ * 180.0 / M_PI,
      k_ff_, lookahead_x_, kappa_blend_, 1.0 - kappa_blend_);
  }

private:
  static constexpr double WHEELBASE_M = 0.21;

  void err_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    e_y_           = msg->x;
    e_psi_         = msg->y;
    kappa_         = msg->z;
    last_err_time_ = this->now();
    has_received_  = true;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    double vx   = msg->twist.twist.linear.x;
    double vyaw = msg->twist.twist.angular.z;
    if (vx > kappa_odom_min_vx_) {
      kappa_odom_ = std::clamp(vyaw / vx, -2.0, 2.0);
      odom_valid_ = true;
    } else {
      odom_valid_ = false;
    }
  }

  void control_loop()
  {
    geometry_msgs::msg::Twist twist;

    if (!has_received_) {
      angular_z_smooth_ = 0.0;
      pub_->publish(twist);
      return;
    }

    double elapsed = (this->now() - last_err_time_).seconds();
    if (elapsed > timeout_) {
      angular_z_smooth_ = 0.0;
      pub_->publish(twist);  // safety stop
      return;
    }

    double v = speed_;

    auto deadband = [](double x, double db) {
      return std::abs(x) < db ? 0.0 : x;
    };

    // ── Camera reference tracking across a lane change ───────────────────
    // e_y_ is measured from whichever lane center the camera currently
    // tracks. When the robot merges into the outer lane, the camera's
    // reference lane center shifts too and e_y_ jumps toward 0 even though
    // the robot hasn't reached the outer lane center yet. Left uncorrected,
    // this reads as "still needs to move left" and drives the robot off the
    // track. Fix: on the reference jump, latch cam_ref_offset_ = the current
    // target_offset and add it back into e_y_ before subtracting
    // target_offset, so the error stays correct in both lanes.
    bool in_overtake = (overtake_state_ == "OVERTAKE");
    bool in_return   = (overtake_state_ == "RETURN");

    if (in_overtake) {
        if (!cam_ref_applied_ && std::abs(e_y_) < 0.15 && std::abs(target_offset_) > 0.30) {
            cam_ref_offset_  = target_offset_;
            cam_ref_applied_ = true;
        }
    } else if (!in_return) {
        cam_ref_offset_  = 0.0;
        cam_ref_applied_ = false;
    }
    // RETURN: keep cam_ref_offset_ from OVERTAKE (camera is still on the outer lane).

    double corrected_e_y = e_y_ + cam_ref_offset_;
    double e_y_in   = deadband(corrected_e_y - target_offset_, db_ey_);
    double e_psi_in = deadband(e_psi_, db_psi_);

    double kappa_use;
    if (odom_valid_) {
      kappa_use = kappa_blend_ * kappa_odom_ + (1.0 - kappa_blend_) * kappa_;
    } else {
      kappa_use = kappa_;
    }

    double delta_ff    = k_ff_ * std::atan(WHEELBASE_M * kappa_use);
    double e_psi_corr  = e_psi_in - lookahead_x_ * kappa_use;

    double v_denom = std::max(v, v_softening_);
    double delta   = delta_ff + e_psi_corr + std::atan2(k_ * e_y_in, v_denom);
    delta          = std::clamp(delta, -max_steer_, max_steer_);

    double raw_az = v * std::tan(delta) / WHEELBASE_M;

    // Asymmetric EMA: converge faster (alpha_conv) when the command is
    // decreasing (exiting a curve) to damp oscillation; smoother (alpha)
    // while it's increasing.
    bool  decelerating = std::abs(raw_az) < std::abs(angular_z_smooth_);
    double alpha_eff   = decelerating ? alpha_conv_ : alpha_;
    angular_z_smooth_  = alpha_eff * raw_az + (1.0 - alpha_eff) * angular_z_smooth_;

    twist.linear.x  = v;
    twist.angular.z = angular_z_smooth_;
    pub_->publish(twist);
  }

  // ── Parameters ───────────────────────────────────────────────────────────
  double speed_{8.0};
  double k_{0.30};
  double max_steer_{0.52};
  double timeout_{0.5};
  double v_softening_{0.20};
  double alpha_{0.40};
  double alpha_conv_{0.70};
  double db_ey_{0.020};
  double db_psi_{0.017};
  double k_ff_{1.00};
  double lookahead_x_{0.923};
  double kappa_blend_{0.60};
  double kappa_odom_min_vx_{0.05};

  // ── State ────────────────────────────────────────────────────────────────
  double       e_y_{0.0};
  double       e_psi_{0.0};
  double       kappa_{0.0};      // vision curvature
  double       kappa_odom_{0.0}; // odometry curvature (vyaw/vx)
  bool         odom_valid_{false};
  double       angular_z_smooth_{0.0};
  rclcpp::Time last_err_time_;
  bool         has_received_{false};
  double       target_offset_{0.0};
  std::string  overtake_state_{"FOLLOW"};
  double       cam_ref_offset_{0.0};
  bool         cam_ref_applied_{false};

  // ── ROS interfaces ───────────────────────────────────────────────────────
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr   sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       sub_odom_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr        sub_offset_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr        sub_speed_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr         sub_state_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr        pub_;
  rclcpp::TimerBase::SharedPtr                                    timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaneControlNode>());
  rclcpp::shutdown();
  return 0;
}
