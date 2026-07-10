#include <Arduino.h>

#include "drive_motor.hpp"
#include "imu_sensor.hpp"
#include "micro_ros_bridge.hpp"
#include "robot_config.hpp"
#include "steering_actuator.hpp"

namespace {

DriveMotor rear_left_motor(robot_config::kLeftMotorPwmPin, robot_config::kLeftMotorDirPin,
                            robot_config::kLeftEncoderPinA, robot_config::kLeftEncoderPinB,
                            robot_config::kEncoderCountsPerRev, robot_config::kDriveKp,
                            robot_config::kDriveKi, robot_config::kDriveKd);

DriveMotor rear_right_motor(robot_config::kRightMotorPwmPin, robot_config::kRightMotorDirPin,
                             robot_config::kRightEncoderPinA, robot_config::kRightEncoderPinB,
                             robot_config::kEncoderCountsPerRev, robot_config::kDriveKp,
                             robot_config::kDriveKi, robot_config::kDriveKd);

SteeringActuator steering(robot_config::kSteerServoPin, robot_config::kSteerServoMinAngleRad,
                           robot_config::kSteerServoMaxAngleRad, robot_config::kSteerServoMinPulseUs,
                           robot_config::kSteerServoMaxPulseUs);

ImuSensor imu;
bool imu_ready = false;
uint32_t last_control_tick_ms = 0;

// Runs at a fixed rate regardless of ROS link state: PID/encoders/servo must keep
// working (and fail safely to zero velocity via the bridge's disconnect handler)
// whether or not the agent is currently connected.
void controlLoopTick(float dt) {
  const JointCommand& command = ros_bridge::latestCommand();

  steering.setAngle(command.steer_angle_rad);
  rear_left_motor.setTargetVelocity(command.rear_left_velocity_rad_s);
  rear_right_motor.setTargetVelocity(command.rear_right_velocity_rad_s);

  rear_left_motor.update(dt);
  rear_right_motor.update(dt);

  JointFeedback feedback;
  feedback.steer_angle_rad = steering.commandedAngle();
  feedback.rear_left_velocity_rad_s = rear_left_motor.measuredVelocity();
  feedback.rear_right_velocity_rad_s = rear_right_motor.measuredVelocity();
  feedback.rear_left_position_rad = rear_left_motor.positionRad();
  feedback.rear_right_position_rad = rear_right_motor.positionRad();

  const ImuSample imu_sample = imu_ready ? imu.read() : ImuSample{};

  ros_bridge::publishFeedback(feedback, imu_sample);
}

}  // namespace

void setup() {
  Serial.begin(115200);

  rear_left_motor.begin();
  rear_right_motor.begin();
  steering.begin();
  imu_ready = imu.begin(robot_config::kImuSdaPin, robot_config::kImuSclPin);

  ros_bridge::begin();
}

void loop() {
  ros_bridge::spinSome();

  const uint32_t now_ms = millis();
  const uint32_t period_ms = static_cast<uint32_t>(robot_config::kControlLoopPeriodS * 1000.0f);
  if (now_ms - last_control_tick_ms >= period_ms) {
    const float dt = (now_ms - last_control_tick_ms) / 1000.0f;
    last_control_tick_ms = now_ms;
    controlLoopTick(dt);
  }
}
