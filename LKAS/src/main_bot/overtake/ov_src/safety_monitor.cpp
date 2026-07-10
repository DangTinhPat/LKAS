#include "../ov_inc/safety_monitor.hpp"
#include <algorithm>
#include <cmath>

void SafetyMonitor::on_scan(sensor_msgs::msg::LaserScan::SharedPtr msg) { latest_scan_ = msg; }

void SafetyMonitor::on_odom(nav_msgs::msg::Odometry::SharedPtr msg)
{
    double vx   = msg->twist.twist.linear.x;
    double vyaw = msg->twist.twist.angular.z;
    v_ego_ = std::abs(vx);
    if (std::abs(vx) > 0.05)
        kappa_ = std::clamp(vyaw / vx, -2.0, 2.0);
}

void SafetyMonitor::on_imu(sensor_msgs::msg::Imu::SharedPtr msg)
{
    a_lateral_ = msg->linear_acceleration.y;
}

// Returns the closest finite return in [a_from, a_to] (rad), or 999.0 (no
// obstacle sentinel) if the sector is empty or out of scan range.
double SafetyMonitor::min_sector(double a_from, double a_to) const
{
    if (!latest_scan_) return 999.0;
    const auto & s = *latest_scan_;
    int n = static_cast<int>(s.ranges.size());
    if (n == 0 || s.angle_increment <= 0) return 999.0;
    if (a_from > a_to) std::swap(a_from, a_to);
    double res = 999.0;
    int i0 = std::clamp((int)((a_from - s.angle_min) / s.angle_increment), 0, n - 1);
    int i1 = std::clamp((int)((a_to   - s.angle_min) / s.angle_increment), 0, n - 1);
    for (int i = i0; i <= i1; ++i) {
        float r = s.ranges[i];
        if (std::isfinite(r) && r > s.range_min && r < s.range_max)
            res = std::min(res, static_cast<double>(r));
    }
    return res;
}

SafetyResult SafetyMonitor::run(const Config & cfg)
{
    SafetyResult out;

    // Curve compensation: on a curve of curvature kappa, an NPC D ahead
    // appears at LiDAR angle theta ~= D*kappa, which can exceed the
    // straight-line front sector. theta_bias re-centers all sectors below
    // toward the curve direction.
    const double D_mid      = (cfg.front_safe_min + cfg.front_detect_range) * 0.5;
    const double max_bias   = 55.0 * M_PI / 180.0;
    const double theta_bias = std::clamp(D_mid * kappa_, -max_bias, max_bias);
    out.theta_bias = theta_bias;

    // Module 1: front sector, union of biased (curve) and unbiased (straight)
    // scans so an NPC is caught whether the track is straight or curving.
    const double half_rad = cfg.front_sector_deg * M_PI / 180.0;
    front_dist_ = std::min(
        min_sector(-half_rad, half_rad),
        min_sector(theta_bias - half_rad, theta_bias + half_rad)
    );
    bool front_ok = (front_dist_ > cfg.front_safe_min) && (front_dist_ < cfg.front_detect_range);
    out.front_dist    = front_dist_;
    out.front_present = front_ok;

    // Module 2: adjacent (outer/left) lane sector, rotated with theta_bias.
    const double deg2r = M_PI / 180.0;
    double adj_lo = std::clamp(45.0 * deg2r - theta_bias,
                                10.0 * deg2r, 70.0 * deg2r);
    double adj_hi = std::clamp(135.0 * deg2r - theta_bias * 0.5,
                                90.0 * deg2r, 165.0 * deg2r);
    double adj_dist = min_sector(adj_lo, adj_hi);
    out.adj_clear = (adj_dist > cfg.adjacent_clear_min);

    // Module 3: gap time = front_dist / closing_speed.
    double v_rel = v_ego_ - cfg.npc_speed;
    if (front_ok && v_rel > 0.05)
        out.gap_ok = (front_dist_ / v_rel) < cfg.gap_time_threshold;

    // Module 4: same-lane distance — lateral filter so a parallel NPC in the
    // adjacent lane isn't mistaken for a same-lane obstacle.
    if (latest_scan_) {
        const auto & s = *latest_scan_;
        int    n      = static_cast<int>(s.ranges.size());
        double sl_dist = 999.0;
        double sl_lo = std::min(theta_bias - 60.0 * M_PI / 180.0, -60.0 * M_PI / 180.0);
        double sl_hi = std::max(theta_bias + 60.0 * M_PI / 180.0,  60.0 * M_PI / 180.0);
        int sl_i0 = std::clamp((int)((sl_lo - s.angle_min) / s.angle_increment), 0, n - 1);
        int sl_i1 = std::clamp((int)((sl_hi - s.angle_min) / s.angle_increment), 0, n - 1);
        for (int i = sl_i0; i <= sl_i1; ++i) {
            float r = s.ranges[i];
            if (!std::isfinite(r) || r <= s.range_min || r >= s.range_max) continue;
            double angle = s.angle_min + i * s.angle_increment;
            double lateral = std::min(
                std::abs(r * std::sin(angle - theta_bias)),
                std::abs(r * std::sin(angle))
            );
            if (lateral < cfg.same_lane_half_width)
                sl_dist = std::min(sl_dist, static_cast<double>(r));
        }
        out.same_lane_dist = sl_dist;
    }

    return out;
}

bool SafetyMonitor::check_abort(const Config & cfg, double dt,
                                 double prev_front_dist, bool in_overtake,
                                 double overtake_offset,
                                 std::string & reason) const
{
    // Abort 1: lateral IMU acceleration spike (Layer 6).
    if (std::abs(a_lateral_) > cfg.imu_ay_limit) {
        reason = "IMU a_y=" + std::to_string(a_lateral_) + " m/s²";
        return true;
    }

    // Abort 2: front obstacle. Once well into OVERTAKE, an inner-lane NPC is
    // expected in front, so only abort on near-certain collision.
    bool expect_inner_npc = in_overtake && (std::abs(overtake_offset) > 0.25);
    double front_abort_thr = expect_inner_npc ? 0.05 : cfg.abort_front_dist;
    if (front_dist_ < front_abort_thr) {
        reason = "front=" + std::to_string(front_dist_) + "m thr=" + std::to_string(front_abort_thr);
        return true;
    }

    if (in_overtake) {
        // Abort 3: NPC braking harder than expected.
        double closing_rate = (prev_front_dist - front_dist_) / std::max(dt, 0.01);
        double expected     = v_ego_ - cfg.npc_speed;
        if (closing_rate > expected + 0.5) {
            reason = "NPC braking closing=" + std::to_string(closing_rate) + " m/s";
            return true;
        }

        // Abort 4: oncoming/obstacle in the lane being merged into.
        const double D_mid    = 2.0;
        const double tb       = std::clamp(D_mid * kappa_, -55.0 * M_PI / 180.0,
                                                            55.0 * M_PI / 180.0);
        double left_close = min_sector(25.0 * M_PI / 180.0 - tb * 0.5,
                                       120.0 * M_PI / 180.0);
        if (left_close < 0.18) {
            reason = "oncoming left=" + std::to_string(left_close) + " m";
            return true;
        }
    }

    return false;
}
