cmake_minimum_required(VERSION 3.8)
project(mcu_agent)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(agent_supervisor STATIC src/agent_supervisor.cpp)
target_include_directories(agent_supervisor PUBLIC include)

add_executable(mcu_agent_node src/mcu_agent_node.cpp)
target_include_directories(mcu_agent_node PRIVATE include)
target_link_libraries(mcu_agent_node agent_supervisor)
ament_target_dependencies(mcu_agent_node rclcpp std_msgs)

install(TARGETS mcu_agent_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch scripts
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
