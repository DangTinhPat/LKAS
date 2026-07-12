#include "micro_ros_bridge.hpp"

#include <Arduino.h>

#include <cstring>

#include <micro_ros_platformio.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/string_functions.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>

#include "robot_config.hpp"

namespace ros_bridge {
namespace {

constexpr size_t kNumJoints = 4;
constexpr uint32_t kPingIntervalMs = 500;  // must stay well above one control period,
                                            // see README "Những điều dễ gây nhầm" for why

enum class State { kWaitingAgent, kAgentAvailable, kConnected, kDisconnected };
State state = State::kWaitingAgent;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

rcl_publisher_t joint_states_pub;
rcl_publisher_t imu_pub;
rcl_subscription_t joint_commands_sub;

sensor_msgs__msg__JointState joint_states_msg;
sensor_msgs__msg__JointState joint_commands_msg;
sensor_msgs__msg__Imu imu_msg;

double joint_states_position[kNumJoints];
double joint_states_velocity[kNumJoints];
double joint_commands_position[kNumJoints];
double joint_commands_velocity[kNumJoints];

JointCommand latest_command;

void initJointStateMessage(sensor_msgs__msg__JointState& msg, double* position_storage,
                            double* velocity_storage) {
  sensor_msgs__msg__JointState__init(&msg);

  rosidl_runtime_c__String__Sequence__init(&msg.name, kNumJoints);
  for (size_t i = 0; i < kNumJoints; ++i) {
    rosidl_runtime_c__String__assign(&msg.name.data[i], robot_config::kJointNames[i]);
  }

  msg.position.data = position_storage;
  msg.position.size = kNumJoints;
  msg.position.capacity = kNumJoints;

  msg.velocity.data = velocity_storage;
  msg.velocity.size = kNumJoints;
  msg.velocity.capacity = kNumJoints;
}

// Looks joints up by name instead of trusting array index: RealRobotSystem::write()
// (main_bot/hardware/real_robot_system.cpp) builds /mcu/joint_commands by iterating
// info_.joints in ros2_control.xacro's declaration order and skipping joints with no
// command interface, which is NOT [rear_left, rear_right, front_left_steer,
// front_right_steer] -- it's whatever order the xacro happens to list joints in.
// Matching by name keeps this correct regardless of that order.
void jointCommandCallback(const void* msg_in) {
  const auto* msg = static_cast<const sensor_msgs__msg__JointState*>(msg_in);

  bool have_left_steer = false;
  bool have_right_steer = false;
  float left_steer_rad = 0.0f;
  float right_steer_rad = 0.0f;

  for (size_t i = 0; i < msg->name.size; ++i) {
    const char* name = msg->name.data[i].data;
    if (strcmp(name, "rear_left_wheel_joint") == 0 && i < msg->velocity.size) {
      latest_command.rear_left_velocity_rad_s = static_cast<float>(msg->velocity.data[i]);
    } else if (strcmp(name, "rear_right_wheel_joint") == 0 && i < msg->velocity.size) {
      latest_command.rear_right_velocity_rad_s = static_cast<float>(msg->velocity.data[i]);
    } else if (strcmp(name, "front_left_steer_joint") == 0 && i < msg->position.size) {
      left_steer_rad = static_cast<float>(msg->position.data[i]);
      have_left_steer = true;
    } else if (strcmp(name, "front_right_steer_joint") == 0 && i < msg->position.size) {
      right_steer_rad = static_cast<float>(msg->position.data[i]);
      have_right_steer = true;
    }
  }

  if (have_left_steer && have_right_steer) {
    latest_command.steer_angle_rad = 0.5f * (left_steer_rad + right_steer_rad);
  } else if (have_left_steer) {
    latest_command.steer_angle_rad = left_steer_rad;
  } else if (have_right_steer) {
    latest_command.steer_angle_rad = right_steer_rad;
  }
}

bool createEntities() {
  allocator = rcl_get_default_allocator();

  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, "lkas_mcu_node", "", &support) != RCL_RET_OK) return false;

  if (rclc_publisher_init_default(&joint_states_pub, &node,
                                   ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
                                   "/mcu/joint_states") != RCL_RET_OK) {
    return false;
  }

  if (rclc_publisher_init_default(&imu_pub, &node,
                                   ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
                                   "/imu") != RCL_RET_OK) {
    return false;
  }

  if (rclc_subscription_init_default(&joint_commands_sub, &node,
                                      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
                                      "/mcu/joint_commands") != RCL_RET_OK) {
    return false;
  }

  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &joint_commands_sub, &joint_commands_msg,
                                      &jointCommandCallback, ON_NEW_DATA) != RCL_RET_OK) {
    return false;
  }

  return true;
}

void destroyEntities() {
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  (void)rcl_publisher_fini(&joint_states_pub, &node);
  (void)rcl_publisher_fini(&imu_pub, &node);
  (void)rcl_subscription_fini(&joint_commands_sub, &node);
  (void)rclc_executor_fini(&executor);
  (void)rcl_node_fini(&node);
  (void)rclc_support_fini(&support);
}

}  // namespace

void begin() {
  initJointStateMessage(joint_states_msg, joint_states_position, joint_states_velocity);
  initJointStateMessage(joint_commands_msg, joint_commands_position, joint_commands_velocity);

  sensor_msgs__msg__Imu__init(&imu_msg);
  rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_link");
  imu_msg.orientation_covariance[0] = -1.0;  // orientation not provided (REP 145 convention)

  set_microros_serial_transports(Serial);
  state = State::kWaitingAgent;
}

// Ping-based health checks are rate-limited (kPingIntervalMs): pinging on every
// loop() iteration contends with the executor for the same serial link and makes the
// agent see a false disconnect roughly once a second.
void spinSome() {
  static uint32_t last_ping_ms = 0;
  const uint32_t now_ms = millis();
  const bool ping_due = now_ms - last_ping_ms > kPingIntervalMs;

  switch (state) {
    case State::kWaitingAgent:
      if (ping_due) {
        last_ping_ms = now_ms;
        state = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) ? State::kAgentAvailable
                                                              : State::kWaitingAgent;
      }
      break;

    case State::kAgentAvailable:
      state = createEntities() ? State::kConnected : State::kWaitingAgent;
      if (state == State::kWaitingAgent) {
        destroyEntities();
      }
      break;

    case State::kConnected:
      if (ping_due) {
        last_ping_ms = now_ms;
        state = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) ? State::kConnected
                                                              : State::kDisconnected;
      }
      if (state == State::kConnected) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
      }
      break;

    case State::kDisconnected:
      destroyEntities();
      latest_command = JointCommand{};  // fail-safe: stop the drivetrain when the link drops
      state = State::kWaitingAgent;
      break;
  }
}

bool isConnected() { return state == State::kConnected; }

const JointCommand& latestCommand() { return latest_command; }

void publishFeedback(const JointFeedback& feedback, const ImuSample& imu) {
  if (state != State::kConnected) {
    return;
  }

  joint_states_velocity[0] = feedback.rear_left_velocity_rad_s;
  joint_states_velocity[1] = feedback.rear_right_velocity_rad_s;
  joint_states_position[0] = feedback.rear_left_position_rad;
  joint_states_position[1] = feedback.rear_right_position_rad;
  joint_states_position[2] = feedback.steer_angle_rad;
  joint_states_position[3] = feedback.steer_angle_rad;
  rcl_publish(&joint_states_pub, &joint_states_msg, NULL);

  imu_msg.angular_velocity.x = imu.angular_velocity[0];
  imu_msg.angular_velocity.y = imu.angular_velocity[1];
  imu_msg.angular_velocity.z = imu.angular_velocity[2];
  imu_msg.linear_acceleration.x = imu.linear_acceleration[0];
  imu_msg.linear_acceleration.y = imu.linear_acceleration[1];
  imu_msg.linear_acceleration.z = imu.linear_acceleration[2];
  rcl_publish(&imu_pub, &imu_msg, NULL);
}

}  // namespace ros_bridge
