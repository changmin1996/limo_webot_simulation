#pragma once

// #include "rclcpp/macros.hpp"
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"


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
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  geometry_msgs::msg::Twist cmd_vel_msg_;

  WbDeviceTag fl_motor_,  fr_motor_,  rl_motor_, rr_motor_;
  WbDeviceTag fl_ps_, fr_ps_, rl_ps_, rr_ps_;

  double x_{0.0}, y_{0.0}, yaw_{0.0};
  double prev_left_pos_{0.0}, prev_right_pos_{0.0};
  bool odom_initialized_{false};

  double wheel_radius_{0.045};
  double half_track_{0.215};
  bool publish_tf_{true};

  webots_ros2_driver::WebotsNode* node_{nullptr};
  double dt_{0.0};
};
}