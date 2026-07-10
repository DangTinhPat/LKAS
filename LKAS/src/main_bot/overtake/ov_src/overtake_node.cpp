// overtake_node.cpp — ROS 2 node for the overtake pipeline (7 layers).
//
// Architecture:
//   SafetyMonitor  : Layers 1-3, 6-7  (sensors + safety modules + abort)
//   StateMachine   : Layer 4          (FOLLOW -> PREPARE -> OVERTAKE -> RETURN)
//   OffsetPlanner  : Layer 5          (rate-limited target_offset per state)
//
// Publishes:
//   /overtake/state         (std_msgs/String)    -> debug
//   /overtake/target_offset (std_msgs/Float64)   -> lane_control_node
//   /overtake/target_speed  (std_msgs/Float64)   -> lane_control_node

#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "../ov_inc/safety_monitor.hpp"
#include "../ov_inc/state_machine.hpp"
#include "../ov_inc/offset_planner.hpp"

using namespace std::chrono_literals;

class OvertakeNode : public rclcpp::Node
{
public:
    OvertakeNode() : Node("overtake_node")
    {
        // SafetyMonitor params
        this->declare_parameter("front_detect_range",  3.0);
        this->declare_parameter("front_safe_min",      0.40);
        this->declare_parameter("front_sector_deg",   30.0);
        this->declare_parameter("adjacent_clear_min",  0.50);
        this->declare_parameter("npc_speed",           0.25);
        this->declare_parameter("gap_time_threshold",  4.0);
        this->declare_parameter("abort_front_dist",      0.35);
        this->declare_parameter("imu_ay_limit",          3.0);
        this->declare_parameter("same_lane_half_width",  0.15);
        // StateMachine params
        this->declare_parameter("prepare_hold_time",   1.0);
        this->declare_parameter("overtake_hold_time",  4.0);
        this->declare_parameter("return_tol",          0.04);
        this->declare_parameter("return_hold_time",    2.0);
        // OffsetPlanner params
        this->declare_parameter("overtake_offset",    -0.534);
        this->declare_parameter("offset_rate_limit",   0.45);
        this->declare_parameter("return_rate_limit",   0.60);
        // Speed control params
        this->declare_parameter("normal_speed",  1.0);
        this->declare_parameter("follow_speed",  0.30);
        this->declare_parameter("creep_speed",   0.15);

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            [this](sensor_msgs::msg::LaserScan::SharedPtr m) {
                monitor_.on_scan(m);
                scan_received_ = true;
            });

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10,
            [this](nav_msgs::msg::Odometry::SharedPtr m) {
                monitor_.on_odom(m);
                odom_received_ = true;
            });

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10,
            [this](sensor_msgs::msg::Imu::SharedPtr m) { monitor_.on_imu(m); });

        vision_dist_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/overtake/vision_front_dist", 10,
            [this](std_msgs::msg::Float32::SharedPtr m) {
                vision_front_dist_ = m->data;
            });

        vision_adj_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/overtake/vision_adj_clear", 10,
            [this](std_msgs::msg::Bool::SharedPtr m) {
                vision_adj_clear_ = m->data;
                vision_received_  = true;
            });

        state_pub_  = this->create_publisher<std_msgs::msg::String>("/overtake/state",  10);
        offset_pub_ = this->create_publisher<std_msgs::msg::Float64>("/overtake/target_offset", 10);
        speed_pub_  = this->create_publisher<std_msgs::msg::Float64>("/overtake/target_speed",  10);

        sm_timer_ = this->create_wall_timer(100ms, std::bind(&OvertakeNode::sm_step, this));
        last_time_ = this->now();

        RCLCPP_INFO(this->get_logger(),
            "[overtake] ready — front_range=%.1fm  gap_thr=%.1fs  offset=%.3fm",
            this->get_parameter("front_detect_range").as_double(),
            this->get_parameter("gap_time_threshold").as_double(),
            this->get_parameter("overtake_offset").as_double());
    }

private:
    void sm_step()
    {
        auto now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        // Params are re-read every cycle to support `ros2 param set` at runtime.
        SafetyMonitor::Config scfg{
            this->get_parameter("front_detect_range").as_double(),
            this->get_parameter("front_safe_min").as_double(),
            this->get_parameter("front_sector_deg").as_double(),
            this->get_parameter("adjacent_clear_min").as_double(),
            this->get_parameter("npc_speed").as_double(),
            this->get_parameter("gap_time_threshold").as_double(),
            this->get_parameter("abort_front_dist").as_double(),
            this->get_parameter("imu_ay_limit").as_double(),
            this->get_parameter("same_lane_half_width").as_double(),
        };
        StateMachine::Config mcfg{
            this->get_parameter("prepare_hold_time").as_double(),
            this->get_parameter("overtake_hold_time").as_double(),
            this->get_parameter("return_tol").as_double(),
            this->get_parameter("return_hold_time").as_double(),
        };
        OffsetPlanner::Config pcfg{
            this->get_parameter("overtake_offset").as_double(),
            this->get_parameter("offset_rate_limit").as_double(),
            this->get_parameter("return_rate_limit").as_double(),
        };

        debug_tick_++;
        if (debug_tick_ <= 50 && debug_tick_ % 10 == 0) {
            if (!scan_received_)
                RCLCPP_WARN(this->get_logger(), "[overtake] no /scan received yet!");
            if (!odom_received_)
                RCLCPP_WARN(this->get_logger(), "[overtake] no /odometry/filtered received yet!");
        }

        // Layer 2-3: safety modules (LiDAR)
        SafetyResult safety = monitor_.run(scfg);

        // Vision only augments front-obstacle detection (long range); it does
        // not gate adjacent-lane clearance. The camera looks forward, not
        // sideways, so a parallel outer-lane NPC constantly appears in frame
        // and would falsely block overtaking. Adjacent clearance stays
        // LiDAR-only (left sector 45-135 deg).
        bool vis_front = (vision_front_dist_ > 0.0f &&
                          vision_front_dist_ < static_cast<float>(scfg.front_detect_range));
        bool fused_front = safety.front_present || vis_front;
        bool fused_adj = safety.adj_clear;

        bool can_overtake = fused_front && fused_adj;

        // Debounce: require 3 consecutive adj_clear=false ticks (~300ms)
        // before cancelling PREPARE, to avoid FOLLOW<->PREPARE oscillation
        // from single-tick sensor noise.
        if (!fused_adj) adj_fail_count_++;
        else            adj_fail_count_ = 0;
        bool can_prepare_hold = (adj_fail_count_ < 3);

        // Layer 6-7: abort check
        bool in_overtake = (sm_.state() == OvertakeState::OVERTAKE);
        std::string abort_reason;
        bool abort = monitor_.check_abort(scfg, dt, prev_front_dist_, in_overtake,
                                          planner_.offset(), abort_reason);
        if (abort) {
            RCLCPP_WARN(this->get_logger(), "[overtake] ABORT: %s", abort_reason.c_str());
        }

        if (can_overtake && !prev_can_overtake_) {
            RCLCPP_WARN(this->get_logger(),
                "[overtake] *** CAN_OVERTAKE = TRUE ***"
                "  L_front=%.2fm(%d)  cam_front=%.2f(%d)  adj=%d  v=%.2f",
                safety.front_dist, static_cast<int>(safety.front_present),
                vision_front_dist_, static_cast<int>(vis_front),
                static_cast<int>(safety.adj_clear),
                monitor_.v_ego());
        }
        prev_can_overtake_ = can_overtake;

        // Layer 4: state machine. right_dist checks whether the NPC has been
        // passed (OVERTAKE -> RETURN); the sector is shifted by theta_bias to
        // track the NPC correctly through curves.
        double tb = safety.theta_bias;
        double r_lo = std::clamp(-135.0 * M_PI / 180.0 - tb, -165.0 * M_PI / 180.0, -60.0 * M_PI / 180.0);
        double r_hi = std::clamp( -45.0 * M_PI / 180.0 - tb, -90.0 * M_PI / 180.0, -10.0 * M_PI / 180.0);
        double right_dist = monitor_.min_sector(r_lo, r_hi);
        bool changed = sm_.update(can_overtake, can_prepare_hold, abort,
                                  right_dist, planner_.offset(), mcfg, now);
        if (changed) {
            RCLCPP_WARN(this->get_logger(), "[overtake] → %s  front=%.2fm  adj=%d  gap=%d",
                sm_.state_name(), safety.front_dist, safety.adj_clear, safety.gap_ok);
        }

        // Layer 5: offset planner (updated before speed control so the new
        // offset is available this same cycle)
        planner_.step(sm_.state(), pcfg, dt);

        // ── Speed control ────────────────────────────────────────────────
        double normal_speed = this->get_parameter("normal_speed").as_double();
        double follow_speed = this->get_parameter("follow_speed").as_double();
        double creep_speed  = this->get_parameter("creep_speed").as_double();

        auto   cur_state   = sm_.state();
        double cur_offset  = planner_.offset();
        double full_offset = this->get_parameter("overtake_offset").as_double();
        double lateral_done = (full_offset != 0.0)
                              ? std::abs(cur_offset / full_offset) : 0.0;

        double target_speed;
        double sl_dist = safety.same_lane_dist;
        bool   sl_present = sl_dist < scfg.front_detect_range;

        double npc_speed = this->get_parameter("npc_speed").as_double();

        if (cur_state == OvertakeState::OVERTAKE) {
            if (lateral_done < 0.7) {
                // Still merging laterally: match NPC speed so the gap doesn't close further.
                target_speed = npc_speed;
            } else {
                // Fully in the outer lane: accelerate past the NPC.
                target_speed = normal_speed;
            }
        } else if (cur_state == OvertakeState::PREPARE) {
            // Match NPC speed to hold a stable gap; only creep when the gap
            // is well inside front_safe_min to avoid oscillating near the
            // threshold.
            target_speed = (sl_dist < scfg.front_safe_min * 0.7) ? creep_speed : npc_speed;
        } else if (cur_state == OvertakeState::RETURN) {
            // follow_speed (not normal_speed) softens the merge back into
            // the inner lane so the camera has time to reacquire it.
            target_speed = follow_speed;
        } else if (sl_dist < scfg.front_safe_min) {
            target_speed = creep_speed;
        } else if (sl_present) {
            // Linear speed ramp between front_safe_min and front_detect_range.
            double d_max = scfg.front_detect_range;
            double d_min = scfg.front_safe_min;
            double t     = std::clamp((sl_dist - d_min) / (d_max - d_min), 0.0, 1.0);
            target_speed = follow_speed + t * (normal_speed - follow_speed);
        } else {
            target_speed = normal_speed;
        }

        if (debug_tick_ % 5 == 0) {
            RCLCPP_WARN(this->get_logger(),
                "[OV] %-8s  front=%.2fm(%d)  sl=%.2fm  adj=%d  v=%.2f  off=%.3f(%.0f%%)  spd=%.2f  tb=%.0fdeg",
                sm_.state_name(),
                safety.front_dist,  static_cast<int>(safety.front_present),
                safety.same_lane_dist,
                static_cast<int>(safety.adj_clear),
                monitor_.v_ego(),
                planner_.offset(), lateral_done * 100.0,
                target_speed,
                safety.theta_bias * 180.0 / M_PI);
        }

        std_msgs::msg::Float64 spd_msg;
        spd_msg.data = target_speed;
        speed_pub_->publish(spd_msg);

        std_msgs::msg::String s_msg;
        s_msg.data = sm_.state_name();
        state_pub_->publish(s_msg);

        std_msgs::msg::Float64 o_msg;
        o_msg.data = planner_.offset();
        offset_pub_->publish(o_msg);

        prev_front_dist_ = safety.front_dist;
    }

    // ── Modules ──────────────────────────────────────────────────────────────
    SafetyMonitor monitor_;
    StateMachine  sm_;
    OffsetPlanner planner_;

    double       prev_front_dist_{999.0};
    bool         prev_can_overtake_{false};
    bool         scan_received_{false};
    bool         odom_received_{false};
    rclcpp::Time last_time_;
    int          debug_tick_{0};
    int          adj_fail_count_{0};

    // Vision fusion state
    float        vision_front_dist_{-1.0f};
    bool         vision_adj_clear_{true};
    bool         vision_received_{false};

    // ── ROS interfaces ───────────────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr     odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr       imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr      vision_dist_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr         vision_adj_sub_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr  state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr offset_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub_;

    rclcpp::TimerBase::SharedPtr sm_timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OvertakeNode>());
    rclcpp::shutdown();
    return 0;
}
