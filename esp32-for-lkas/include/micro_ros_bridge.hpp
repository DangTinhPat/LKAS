#pragma once

#include "robot_types.hpp"

// Owns every rcl/rclc entity and the connect/wait/reconnect state machine. Callers
// never touch micro-ROS types directly -- they read the latest command and hand over
// plain sensor readings to publish.
namespace ros_bridge {

void begin();
void spinSome();
bool isConnected();

const JointCommand& latestCommand();
void publishFeedback(const JointFeedback& feedback, const ImuSample& imu);

}  // namespace ros_bridge
