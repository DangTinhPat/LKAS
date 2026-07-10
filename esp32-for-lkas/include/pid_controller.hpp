#pragma once

// Standard clamped PID with anti-windup, used for closed-loop wheel velocity control.
class PIDController {
 public:
  PIDController(float kp, float ki, float kd, float output_min, float output_max);

  float update(float setpoint, float measurement, float dt);
  void reset();

 private:
  float kp_;
  float ki_;
  float kd_;
  float output_min_;
  float output_max_;

  float integral_ = 0.0f;
  float prev_error_ = 0.0f;
};
