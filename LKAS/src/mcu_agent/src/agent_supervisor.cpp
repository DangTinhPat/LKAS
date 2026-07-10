#include "mcu_agent/agent_supervisor.hpp"

#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <utility>
#include <vector>

extern char ** environ;

namespace mcu_agent
{

AgentSupervisor::AgentSupervisor(Options options) : options_(std::move(options)) {}

AgentSupervisor::~AgentSupervisor() { stop(); }

void AgentSupervisor::start()
{
  if (poll_running()) {
    return;
  }

  // Equivalent to: ros2 run micro_ros_agent <agent_executable> serial --dev <port> -b <baud>
  std::vector<std::string> args = {
    "ros2", "run", "micro_ros_agent", options_.agent_executable,
    "serial", "--dev", options_.serial_port, "-b", std::to_string(options_.baud_rate),
  };

  std::vector<char *> argv;
  argv.reserve(args.size() + 1);
  for (auto & a : args) {
    argv.push_back(a.data());
  }
  argv.push_back(nullptr);

  pid_t pid = -1;
  const int rc = posix_spawnp(&pid, "ros2", nullptr, nullptr, argv.data(), environ);
  child_pid_ = (rc == 0) ? pid : -1;
}

void AgentSupervisor::stop()
{
  if (child_pid_ <= 0) {
    return;
  }

  kill(child_pid_, SIGTERM);
  for (int i = 0; i < 20 && poll_running(); ++i) {
    usleep(100 * 1000);  // 100ms, ~2s total grace period
  }
  if (child_pid_ > 0) {
    kill(child_pid_, SIGKILL);
    int status = 0;
    waitpid(child_pid_, &status, 0);
    child_pid_ = -1;
  }
}

bool AgentSupervisor::poll_running()
{
  if (child_pid_ <= 0) {
    return false;
  }
  int status = 0;
  const pid_t r = waitpid(child_pid_, &status, WNOHANG);
  if (r == 0) {
    return true;
  }
  child_pid_ = -1;
  return false;
}

}  // namespace mcu_agent
