#pragma once

#include <ESP32Encoder.h>
#include <stdint.h>

#include "pid_controller.hpp"

// One rear driven wheel: quadrature encoder feedback + PWM/DIR motor output, closed
// over a velocity PID. Two instances of this class are the entire drivetrain.
class DriveMotor {
 public:
  DriveMotor(int pwm_pin, int dir_pin, int encoder_pin_a, int encoder_pin_b,
             float counts_per_rev, float kp, float ki, float kd);

  void begin();
  void setTargetVelocity(float rad_per_s);
  void update(float dt);

  float measuredVelocity() const { return measured_velocity_rad_s_; }
  float positionRad() const { return position_rad_; }

 private:
  void writeMotor(float signed_duty);

  int pwm_pin_;
  int dir_pin_;
  int encoder_pin_a_;
  int encoder_pin_b_;
  float counts_per_rev_;

  ESP32Encoder encoder_;
  PIDController pid_;

  float target_velocity_rad_s_ = 0.0f;
  float measured_velocity_rad_s_ = 0.0f;
  float position_rad_ = 0.0f;
  int64_t last_encoder_count_ = 0;
};
