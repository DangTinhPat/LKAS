#include "../ov_inc/offset_planner.hpp"
#include <algorithm>
#include <cmath>

void OffsetPlanner::step(OvertakeState state, const Config & cfg, double dt)
{
    goal_ = (state == OvertakeState::OVERTAKE) ? cfg.overtake_offset : 0.0;

    // Hồi phục về làn gốc dùng rate nhanh hơn → dứt khoát, ít lag
    double rate      = (state == OvertakeState::RETURN) ? cfg.return_rate_limit
                                                        : cfg.offset_rate_limit;
    double max_delta = rate * std::max(dt, 0.001);
    double diff      = goal_ - offset_;

    if (std::abs(diff) <= max_delta)
        offset_ = goal_;
    else
        offset_ += (diff > 0) ? max_delta : -max_delta;
}
