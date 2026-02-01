#pragma once

// #include "rclcpp/macros.hpp"
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace limo_webot_simulation{
class LimoDiffDriver : public webots_ros2_driver::PluginInterface
{
public:
  void step() override;
  void init(webots_ros2_driver::WebotsNode *node,
            std::unordered_map<std::string, std::string> &parameters) override;
private:
  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  geometry_msgs::msg::Twist cmd_vel_msg_;
  WbDeviceTag fl_motor_;
  WbDeviceTag fr_motor_;
  WbDeviceTag rl_motor_;
  WbDeviceTag rr_motor_;
};
}