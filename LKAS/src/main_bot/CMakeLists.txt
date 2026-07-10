cmake_minimum_required(VERSION 3.8)
project(main_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# ── C++ nodes ────────────────────────────────────────────────────────────────
add_executable(lane_control_node src/lane_control_node.cpp)
ament_target_dependencies(lane_control_node rclcpp geometry_msgs nav_msgs std_msgs)

add_executable(npc_driver_node worlds/npc/npc_driver_node.cpp)
ament_target_dependencies(npc_driver_node rclcpp)
target_link_libraries(npc_driver_node
  gz-transport13::core
  gz-msgs10::core
)

add_executable(overtake_node
    overtake/ov_src/overtake_node.cpp
    overtake/ov_src/safety_monitor.cpp
    overtake/ov_src/state_machine.cpp
    overtake/ov_src/offset_planner.cpp
)
ament_target_dependencies(overtake_node rclcpp sensor_msgs nav_msgs std_msgs)

install(TARGETS lane_control_node npc_driver_node overtake_node
  DESTINATION lib/${PROJECT_NAME}
)

# ── ros2_control hardware plugin (real robot) ───────────────────────────────
add_library(main_bot_hardware SHARED hardware/real_robot_system.cpp)
target_include_directories(main_bot_hardware PRIVATE hardware/include)
ament_target_dependencies(main_bot_hardware
  rclcpp rclcpp_lifecycle hardware_interface pluginlib sensor_msgs
)
pluginlib_export_plugin_description_file(hardware_interface hardware/main_bot_hardware.xml)

install(TARGETS main_bot_hardware
  DESTINATION lib
)
install(
  DIRECTORY hardware/include/
  DESTINATION include
)

# ── Share assets ─────────────────────────────────────────────────────────────
install(
  DIRECTORY launch config description worlds models
  DESTINATION share/${PROJECT_NAME}
)

# ── Python nodes ─────────────────────────────────────────────────────────────
install(
  PROGRAMS
    scripts/odom_tf_relay.py
    scripts/lane_follower_node.py
    scripts/overtake_vision_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY scripts/vision
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
