#pragma once
#include <rclcpp/time.hpp>

// Layer 4 of the overtake pipeline: FOLLOW -> PREPARE -> OVERTAKE -> RETURN.
// See overtake_node.cpp for the full pipeline diagram.

enum class OvertakeState { FOLLOW, PREPARE, OVERTAKE, RETURN };

class StateMachine
{
public:
    struct Config {
        double prepare_hold_time{1.0};
        double overtake_hold_time{4.0};
        double return_tol{0.04};
        double return_hold_time{2.0};
    };

    bool update(bool can_overtake, bool can_prepare_hold, bool abort,
                double right_dist, double cur_offset,
                const Config & cfg, const rclcpp::Time & now);

    OvertakeState state()      const { return state_; }
    const char *  state_name() const;

private:
    OvertakeState state_{OvertakeState::FOLLOW};
    rclcpp::Time  state_entry_time_{0, 0, RCL_ROS_TIME};  // must match node's use_sim_time clock

    void   transition_to(OvertakeState s, const rclcpp::Time & now);
    double time_in_state(const rclcpp::Time & now) const;
};
