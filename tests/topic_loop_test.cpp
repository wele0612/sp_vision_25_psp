#include <geometry_msgs/msg/twist.hpp>
#include <thread>

#include "io/ros2/ros2.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"

int main(int argc, char ** argv)
{
  tools::Exiter exiter;
  io::ROS2 ros2;
  auto twist_publisher = ros2.create_publisher<geometry_msgs::msg::Twist>(
    "temp_node", "/cmd_vel_smoothed", 10);

  int i = 0;
  while (!exiter.exit()) {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = 1.0F;
    msg.linear.y = -0.5F;
    twist_publisher->publish(msg);

    i++;
    std::this_thread::sleep_for(std::chrono::microseconds(5));

    if (i % 3 == 0) {
      const auto twist = ros2.subscribe_twist();
      if (twist.size() == 2) {
        tools::logger()->info("linear.x: {}, linear.y: {}", twist[0], twist[1]);
      }
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
    if (i > 1000) break;
  }

  return 0;
}
