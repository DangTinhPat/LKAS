#pragma once
#include <memory>
#include <string>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

// Layers 2-3 and 6-7 of the overtake pipeline: LiDAR/IMU/odometry sensor
// fusion, safety-module evaluation (front/adjacent/gap/same-lane) and
// abort-condition checks. See overtake_node.cpp for the full pipeline.

struct SafetyResult {
    double front_dist{999.0};
    bool   front_present{false};
    bool   adj_clear{true};
    bool   gap_ok{false};
    double theta_bias{0.0};       // curvature bias [rad], consumed by overtake_node
    double same_lane_dist{999.0}; // nearest same-lane obstacle [m], used for speed control
};

class SafetyMonitor
{
public:
    struct Config {
        double front_detect_range{3.0};
        double front_safe_min{0.40};
        double front_sector_deg{30.0};
        double adjacent_clear_min{0.50};
        double npc_speed{0.25};
        double gap_time_threshold{4.0};
        double abort_front_dist{0.35};
        double imu_ay_limit{3.0};
        double same_lane_half_width{0.15};
    };

    void on_scan(sensor_msgs::msg::LaserScan::SharedPtr msg);
    void on_odom(nav_msgs::msg::Odometry::SharedPtr msg);
    void on_imu(sensor_msgs::msg::Imu::SharedPtr msg);

    SafetyResult run(const Config & cfg);

    bool check_abort(const Config & cfg, double dt,
                     double prev_front_dist, bool in_overtake,
                     double overtake_offset,
                     std::string & reason) const;

    double min_sector(double a_from, double a_to) const;

    double front_dist() const { return front_dist_; }
    double v_ego()      const { return v_ego_; }
    double kappa()      const { return kappa_; }

private:
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    double v_ego_{0.0};
    double a_lateral_{0.0};
    double front_dist_{999.0};
    double kappa_{0.0};   // vyaw/vx [rad/m], positive = left curve
};
