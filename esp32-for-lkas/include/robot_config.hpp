#pragma once

// Pin numbers, gains, and calibration values for this specific robot. Every constant
// below must be adjusted to match the actual wiring, encoder resolution, gearing, and
// servo throw before the control behavior can be trusted — nothing here is derived
// from the real hardware, only reasonable placeholders for a 1 servo + 2 driven-wheel
// Ackermann chassis wired to typical PWM+DIR motor drivers.

namespace robot_config {

// --- Steering servo: 1 servo, 1:1 linkage to both front wheels ---
constexpr int kSteerServoPin = 13;
constexpr float kSteerMaxAngleRad = 0.52f;  // matches the ros2_control clamp, see mcu_agent/README.md
constexpr float kSteerServoMinAngleRad = -kSteerMaxAngleRad;
constexpr float kSteerServoMaxAngleRad = kSteerMaxAngleRad;
constexpr int kSteerServoMinPulseUs = 1000;
constexpr int kSteerServoMaxPulseUs = 2000;

// --- Rear-left drive motor + encoder ---
// GPIO26-37 are reserved for the octal PSRAM bus on this N16R8 module -- never
// route anything through them. See HARDWARE.md for the full safe/reserved pin map.
constexpr int kLeftMotorPwmPin = 4;
constexpr int kLeftMotorDirPin = 5;
constexpr int kLeftEncoderPinA = 6;
constexpr int kLeftEncoderPinB = 7;

// --- Rear-right drive motor + encoder ---
constexpr int kRightMotorPwmPin = 15;
constexpr int kRightMotorDirPin = 16;
constexpr int kRightEncoderPinA = 17;
constexpr int kRightEncoderPinB = 18;

// --- Shared drive characteristics ---
constexpr float kEncoderCountsPerRev = 1980.0f;  // encoder CPR * gearbox ratio -- measure on the real motor
constexpr float kMotorPwmFrequencyHz = 20000.0f;
constexpr int kMotorPwmResolutionBits = 10;
constexpr int kMotorPwmMaxDuty = (1 << kMotorPwmResolutionBits) - 1;

// Velocity PID: error is rad/s, output is signed PWM duty clamped to +-kMotorPwmMaxDuty.
constexpr float kDriveKp = 40.0f;
constexpr float kDriveKi = 120.0f;
constexpr float kDriveKd = 0.0f;

// --- IMU (MPU6050 over I2C) ---
constexpr int kImuSdaPin = 8;
constexpr int kImuSclPin = 9;

// --- Control loop driving PID + actuator writes, independent of the ROS link state ---
constexpr float kControlLoopHz = 50.0f;
constexpr float kControlLoopPeriodS = 1.0f / kControlLoopHz;

// Order and spelling are fixed by the topic contract in LKAS/src/mcu_agent/README.md —
// RealRobotSystem on the PC side indexes joints by this exact order.
constexpr const char* kJointNames[4] = {
    "rear_left_wheel_joint",
    "rear_right_wheel_joint",
    "front_left_steer_joint",
    "front_right_steer_joint",
};

}  // namespace robot_config
