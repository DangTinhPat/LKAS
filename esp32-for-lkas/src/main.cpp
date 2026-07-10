#include <Arduino.h>
#include <string.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/string.h>

// Connectivity test only: publishes the fixed string "tran_phuong_anh" once a
// second on /esp32_test_topic, to verify the USB-serial link to micro_ros_agent
// (LKAS/src/mcu_agent) is working end-to-end.
//
// On the PC side, after starting mcu_agent, check with:
//   ros2 topic echo /esp32_test_topic
//
// Runs the standard micro-ROS wait/connect/disconnect state machine instead of
// a one-shot setup(): the agent is rarely already listening the instant the
// board boots (or a USB blip can drop the session later), so a single failed
// handshake must not be fatal — the board keeps retrying instead of getting
// stuck forever.

rcl_publisher_t publisher;
std_msgs__msg__String msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

enum State { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
State state = WAITING_AGENT;

// Ping-based health checks must be rate-limited: pinging every loop() call
// (every ~100ms) contends with the executor for the same single serial link
// and made the agent see a false disconnect roughly every second in testing.
#define EXECUTE_EVERY_N_MS(MS, X) \
  do { \
    static int64_t init_ms = -1; \
    if (init_ms == -1) { init_ms = millis(); } \
    if ((int64_t)millis() - init_ms > (MS)) { \
      X; \
      init_ms = millis(); \
    } \
  } while (0)

#define RCCHECK(fn) \
  { \
    rcl_ret_t rc = fn; \
    if (rc != RCL_RET_OK) { \
      return false; \
    } \
  }

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer != NULL) {
    rcl_publish(&publisher, &msg, NULL);
    Serial.println("published: tran_phuong_anh");
  }
}

bool create_entities() {
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_test_node", "", &support));

  RCCHECK(rclc_publisher_init_default(
    &publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "esp32_test_topic"));

  RCCHECK(rclc_timer_init_default(
    &timer, &support, RCL_MS_TO_NS(1000), timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  static char message_data[] = "henlo dvt";
  msg.data.data = message_data;
  msg.data.size = strlen(message_data);
  msg.data.capacity = sizeof(message_data);

  return true;
}

void destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&publisher, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  state = WAITING_AGENT;
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, {
        state = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) ? AGENT_AVAILABLE : WAITING_AGENT;
      });
      break;

    case AGENT_AVAILABLE:
      state = create_entities() ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      }
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(500, {
        state = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
      });
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;

    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
  }

  delay(10);
}
