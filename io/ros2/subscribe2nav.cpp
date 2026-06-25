#include "subscribe2nav.hpp"

#include <sstream>
#include <vector>

namespace io
{

Subscribe2Nav::Subscribe2Nav()
: Node("nav_subscriber"),
  twist_queue_(1),
  state_queue_(1),
  twist_counter_(0)
{

  twist_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/aft_cmd_vel", 10,
    std::bind(&Subscribe2Nav::twist_callback, this, std::placeholders::_1));

  state_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
    "/slow_state", 10,
    std::bind(&Subscribe2Nav::state_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "nav_subscriber node initialized.");
}

Subscribe2Nav::~Subscribe2Nav()
{
  RCLCPP_INFO(this->get_logger(), "nav_subscriber node shutting down.");
}

void Subscribe2Nav::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  twist_queue_.clear();
  twist_queue_.push(*msg);

  twist_counter_++;

  if (twist_counter_ >= 2) {
    if (twist_timer_) {
      twist_timer_->cancel();
    }
    twist_timer_ = this->create_wall_timer(std::chrono::milliseconds(600), [this]() {
      twist_queue_.clear();
      twist_counter_ = 0;
      RCLCPP_INFO(
        this->get_logger(), "Twist queue cleared due to inactivity after two messages.");
    });
  }
}

void Subscribe2Nav::state_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  state_queue_.clear();
  state_queue_.push(*msg);
}

void Subscribe2Nav::start()
{
  RCLCPP_INFO(this->get_logger(), "nav_subscriber node Starting to spin...");
  rclcpp::spin(this->shared_from_this());
}

std::vector<float> Subscribe2Nav::subscribe_twist()
{
  if (twist_queue_.empty()) {
    return std::vector<float>();
  }
  geometry_msgs::msg::Twist msg;

  twist_queue_.back(msg);

  return {static_cast<float>(msg.linear.x), static_cast<float>(msg.linear.y)};
}

int Subscribe2Nav::state_subscribe_()
{
  if (state_queue_.empty()) {
    return 0;
  }
  std_msgs::msg::Int32 msg;

  state_queue_.back(msg);

  return msg.data;
}

}  // namespace io
