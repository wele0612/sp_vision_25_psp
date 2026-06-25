#ifndef IO__SUBSCRIBE2NAV_HPP
#define IO__SUBSCRIBE2NAV_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
class Subscribe2Nav : public rclcpp::Node
{
public:
  Subscribe2Nav();

  ~Subscribe2Nav();

  void start();

  std::vector<float> subscribe_twist();

  int state_subscribe_();

private:
  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  void state_callback(const std_msgs::msg::Int32::SharedPtr msg);

  int twist_counter_;

  rclcpp::TimerBase::SharedPtr twist_timer_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscription_;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr state_subscription_;

  tools::ThreadSafeQueue<geometry_msgs::msg::Twist> twist_queue_;

  tools::ThreadSafeQueue<std_msgs::msg::Int32> state_queue_;
};
}  // namespace io

#endif  // IO__SUBSCRIBE2NAV_HPP
