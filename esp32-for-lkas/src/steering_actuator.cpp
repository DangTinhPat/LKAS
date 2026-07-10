#include "steering_actuator.hpp"

#include <algorithm>

SteeringActuator::SteeringActuator(int pin, float min_angle_rad, float max_angle_rad,
                                    int min_pulse_us, int max_pulse_us)
    : pin_(pin),
      min_angle_rad_(min_angle_rad),
      max_angle_rad_(max_angle_rad),
      min_pulse_us_(min_pulse_us),
      max_pulse_us_(max_pulse_us) {}

void SteeringActuator::begin() {
  servo_.setPeriodHertz(50);
  servo_.attach(pin_, min_pulse_us_, max_pulse_us_);
  setAngle(0.0f);
}

// The servo has no position feedback, so the last commanded angle is what gets
// reported back to ROS 2 as this joint's measured position (open-loop approximation).
void SteeringActuator::setAngle(float angle_rad) {
  commanded_angle_rad_ = std::clamp(angle_rad, min_angle_rad_, max_angle_rad_);

  const float span = max_angle_rad_ - min_angle_rad_;
  const float normalized = (commanded_angle_rad_ - min_angle_rad_) / span;
  const int pulse_us = min_pulse_us_ + static_cast<int>(normalized * (max_pulse_us_ - min_pulse_us_));

  servo_.writeMicroseconds(pulse_us);
}
