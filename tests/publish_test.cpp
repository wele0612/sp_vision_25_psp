#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "io/ros2/ros2.hpp"
#include <sentry_msg/msg/sentry_msg.hpp>
#include "tasks/auto_aim/armor.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"

int main(int argc, char ** argv)
{
  tools::Exiter exiter;
  io::ROS2 ros2;

  double i = 0;
  while (!exiter.exit()) {
    sentry_msg::msg::SentryMsg data;
    data.self_hp = i + 1;
    data.match_started = 0;
    ros2.publish(data);
    i++;

    std::this_thread::sleep_for(std::chrono::seconds(1));
    if (i > 1000) break;
  }
  return 0;
}
