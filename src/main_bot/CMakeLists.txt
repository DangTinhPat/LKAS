cmake_minimum_required(VERSION 3.8)
project(main_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# ── C++ nodes ────────────────────────────────────────────────────────────────
add_executable(lane_control_node src/lane_control_node.cpp)
ament_target_dependencies(lane_control_node rclcpp geometry_msgs)

install(TARGETS lane_control_node
  DESTINATION lib/${PROJECT_NAME}
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
