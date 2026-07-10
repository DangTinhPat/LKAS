#include "drive_motor.hpp"

#include <Arduino.h>
#include <math.h>

#include "robot_config.hpp"

DriveMotor::DriveMotor(int pwm_pin, int dir_pin, int encoder_pin_a, int encoder_pin_b,
                        float counts_per_rev, float kp, float ki, float kd)
    : pwm_pin_(pwm_pin),
      dir_pin_(dir_pin),
      encoder_pin_a_(encoder_pin_a),
      encoder_pin_b_(encoder_pin_b),
      counts_per_rev_(counts_per_rev),
      pid_(kp, ki, kd, -robot_config::kMotorPwmMaxDuty, robot_config::kMotorPwmMaxDuty) {}

// Hardware is touched here, not in the constructor: global DriveMotor instances are
// constructed before the Arduino runtime finishes bringing up peripherals, so pinMode/
// ledc/PCNT calls must wait until begin() runs from setup().
void DriveMotor::begin() {
  pinMode(dir_pin_, OUTPUT);
  ledcAttach(pwm_pin_, robot_config::kMotorPwmFrequencyHz, robot_config::kMotorPwmResolutionBits);

  encoder_.attachHalfQuad(encoder_pin_a_, encoder_pin_b_);
  encoder_.clearCount();
  last_encoder_count_ = 0;
}

void DriveMotor::setTargetVelocity(float rad_per_s) { target_velocity_rad_s_ = rad_per_s; }

void DriveMotor::update(float dt) {
  if (dt <= 0.0f) {
    return;
  }

  const int64_t count = encoder_.getCount();
  const int64_t delta = count - last_encoder_count_;
  last_encoder_count_ = count;

  const float delta_rad = (static_cast<float>(delta) / counts_per_rev_) * 2.0f * static_cast<float>(M_PI);
  position_rad_ += delta_rad;
  measured_velocity_rad_s_ = delta_rad / dt;

  const float duty = pid_.update(target_velocity_rad_s_, measured_velocity_rad_s_, dt);
  writeMotor(duty);
}

void DriveMotor::writeMotor(float signed_duty) {
  digitalWrite(dir_pin_, signed_duty >= 0.0f ? HIGH : LOW);
  ledcWrite(pwm_pin_, static_cast<int>(fabsf(signed_duty)));
}
