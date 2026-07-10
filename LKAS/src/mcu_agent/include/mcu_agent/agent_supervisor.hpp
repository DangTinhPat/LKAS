#pragma once

#include <string>
#include <sys/types.h>

namespace mcu_agent
{

// Spawns and supervises the `micro_ros_agent` process — the standard micro-ROS /
// Micro-XRCE-DDS Agent that bridges the ESP32's XRCE-DDS session onto normal ROS 2 topics.
// This class only owns process lifecycle (start/monitor/respawn); the DDS bridging itself is
// done entirely by the vendored micro_ros_agent executable (see scripts/setup_micro_ros_agent.sh
// — it is not part of this package and must be built once, separately).
class AgentSupervisor
{
public:
  struct Options
  {
    std::string serial_port = "/dev/ttyACM0";
    int baud_rate = 115200;
    std::string agent_executable = "micro_ros_agent";
  };

  explicit AgentSupervisor(Options options);
  ~AgentSupervisor();

  AgentSupervisor(const AgentSupervisor &) = delete;
  AgentSupervisor & operator=(const AgentSupervisor &) = delete;

  // Starts the child process if one isn't already running. No-op otherwise.
  void start();

  // Sends SIGTERM, waits up to ~2s, then SIGKILL if still alive. Safe to call when not running.
  void stop();

  // Non-blocking: reaps the child if it has exited. Returns true if a child is currently running.
  bool poll_running();

private:
  Options options_;
  pid_t child_pid_ = -1;
};

}  // namespace mcu_agent
