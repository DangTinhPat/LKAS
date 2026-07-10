#pragma once

#include <ESP32Servo.h>

// One physical servo, tied 1:1 to both front wheels through the steering linkage —
// the firmware only ever handles a single steer angle.
class SteeringActuator {
 public:
  SteeringActuator(int pin, float min_angle_rad, float max_angle_rad, int min_pulse_us,
                    int max_pulse_us);

  void begin();
  void setAngle(float angle_rad);
  float commandedAngle() const { return commanded_angle_rad_; }

 private:
  Servo servo_;
  int pin_;
  float min_angle_rad_;
  float max_angle_rad_;
  int min_pulse_us_;
  int max_pulse_us_;
  float commanded_angle_rad_ = 0.0f;
};
