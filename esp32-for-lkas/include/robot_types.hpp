#pragma once

// Plain data shared between the micro-ROS bridge and the actuator/sensor modules.
// Deliberately has no Arduino/micro-ROS/library dependency so every other header can
// include it without pulling in unrelated build dependencies.

struct JointCommand {
  float steer_angle_rad = 0.0f;
  float rear_left_velocity_rad_s = 0.0f;
  float rear_right_velocity_rad_s = 0.0f;
};

struct JointFeedback {
  float steer_angle_rad = 0.0f;
  float rear_left_velocity_rad_s = 0.0f;
  float rear_right_velocity_rad_s = 0.0f;
  float rear_left_position_rad = 0.0f;
  float rear_right_position_rad = 0.0f;
};

struct ImuSample {
  float angular_velocity[3] = {0.0f, 0.0f, 0.0f};     // rad/s
  float linear_acceleration[3] = {0.0f, 0.0f, 0.0f};  // m/s^2
};
