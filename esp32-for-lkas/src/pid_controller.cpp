#include "pid_controller.hpp"

PIDController::PIDController(float kp, float ki, float kd, float output_min, float output_max)
    : kp_(kp), ki_(ki), kd_(kd), output_min_(output_min), output_max_(output_max) {}

float PIDController::update(float setpoint, float measurement, float dt) {
  if (dt <= 0.0f) {
    return 0.0f;
  }

  const float error = setpoint - measurement;
  integral_ += error * dt;
  const float derivative = (error - prev_error_) / dt;
  prev_error_ = error;

  float output = kp_ * error + ki_ * integral_ + kd_ * derivative;

  // Anti-windup: undo the integration step whenever it pushed the output past the
  // clamp, so the integral term can't keep growing while saturated.
  if (output > output_max_) {
    output = output_max_;
    integral_ -= error * dt;
  } else if (output < output_min_) {
    output = output_min_;
    integral_ -= error * dt;
  }

  return output;
}

void PIDController::reset() {
  integral_ = 0.0f;
  prev_error_ = 0.0f;
}
