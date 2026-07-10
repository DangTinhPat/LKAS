#pragma once
#include "state_machine.hpp"

// Layer 5 of the overtake pipeline: rate-limited lateral offset target.

class OffsetPlanner
{
public:
    struct Config {
        double overtake_offset{-0.534};
        double offset_rate_limit{0.45};   // m/s, moving to the outer lane
        double return_rate_limit{0.60};   // m/s, returning to the origin lane
    };

    void step(OvertakeState state, const Config & cfg, double dt);

    double offset() const { return offset_; }

private:
    double offset_{0.0};
    double goal_{0.0};
};
