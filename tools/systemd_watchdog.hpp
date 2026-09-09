#pragma once

#include <charconv>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>
#include <string_view>

#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>

namespace tools
{

class SystemdWatchdog
{
public:
  SystemdWatchdog()
  {
    const char * notify_socket = std::getenv("NOTIFY_SOCKET");
    if (notify_socket == nullptr || notify_socket[0] == '\0') return;
    notify_socket_ = notify_socket;

    const char * watchdog_usec = std::getenv("WATCHDOG_USEC");
    if (watchdog_usec == nullptr) return;

    std::uint64_t usec = 0;
    const auto * end = watchdog_usec + std::strlen(watchdog_usec);
    const auto result = std::from_chars(watchdog_usec, end, usec);
    if (result.ec != std::errc{} || result.ptr != end || usec == 0) return;

    const char * watchdog_pid = std::getenv("WATCHDOG_PID");
    if (watchdog_pid != nullptr) {
      pid_t pid = 0;
      const auto * pid_end = watchdog_pid + std::strlen(watchdog_pid);
      const auto pid_result = std::from_chars(watchdog_pid, pid_end, pid);
      if (
        pid_result.ec != std::errc{} || pid_result.ptr != pid_end ||
        pid != ::getpid()) {
        return;
      }
    }

    enabled_ = true;
    interval_ = std::chrono::microseconds(usec / 2);
  }

  ~SystemdWatchdog() { stopping(); }

  bool ready(std::string_view status = {})
  {
    std::string message = "READY=1";
    if (!status.empty()) {
      message += "\nSTATUS=";
      message += status;
    }
    next_ping_ = std::chrono::steady_clock::now();
    return notify(message);
  }

  bool ping()
  {
    if (!enabled_) return true;
    const auto now = std::chrono::steady_clock::now();
    if (now < next_ping_) return true;
    if (!notify("WATCHDOG=1")) return false;
    next_ping_ = now + interval_;
    return true;
  }

  bool stopping() const { return notify("STOPPING=1"); }

private:
  bool notify(std::string_view state) const
  {
    if (notify_socket_.empty()) return true;

    const int fd = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if (fd < 0) return false;

    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    if (notify_socket_.size() >= sizeof(address.sun_path)) {
      ::close(fd);
      return false;
    }

    std::memcpy(address.sun_path, notify_socket_.data(), notify_socket_.size());
    if (address.sun_path[0] == '@') address.sun_path[0] = '\0';
    const auto address_length = static_cast<socklen_t>(
      offsetof(sockaddr_un, sun_path) + notify_socket_.size());
    const auto sent = ::sendto(
      fd, state.data(), state.size(), MSG_NOSIGNAL,
      reinterpret_cast<const sockaddr *>(&address), address_length);
    ::close(fd);
    return sent == static_cast<ssize_t>(state.size());
  }

  std::string notify_socket_;
  bool enabled_ = false;
  std::chrono::microseconds interval_{0};
  std::chrono::steady_clock::time_point next_ping_{};
};

}  // namespace tools
