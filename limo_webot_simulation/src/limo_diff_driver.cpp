#include "limo_webot_simulation/limo_diff_driver.hpp"

#include "rclcpp/rclcpp.hpp"
#include <cstdio>
#include <functional>
#include <webots/motor.h>
#include <webots/robot.h>

#define HALF_DISTANCE_BETWEEN_WHEELS 0.215
#define WHEEL_RADIUS 0.045

namespace limo_webot_simulation
{
void LimoDiffDriver::init(
  webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string>& parameters)
{
  fl_motor_ = wb_robot_get_device("front_left_wheel_joint");
  fr_motor_ = wb_robot_get_device("front_right_wheel_joint");
  rl_motor_ = wb_robot_get_device("rear_left_wheel_joint");
  rr_motor_ = wb_robot_get_device("rear_right_wheel_joint");

  wb_motor_set_position(fl_motor_, INFINITY);
  wb_motor_set_velocity(fl_motor_, 0.0);
  wb_motor_set_position(fr_motor_, INFINITY);
  wb_motor_set_velocity(fr_motor_, 0.0);
  wb_motor_set_position(rl_motor_, INFINITY);
  wb_motor_set_velocity(rl_motor_, 0.0);
  wb_motor_set_position(rr_motor_, INFINITY);
  wb_motor_set_velocity(rr_motor_, 0.0);

  wb_motor_set_acceleration(fl_motor_, -1);
  wb_motor_set_acceleration(fr_motor_, -1);
  wb_motor_set_acceleration(rl_motor_, -1);
  wb_motor_set_acceleration(rr_motor_, -1);

  cmd_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::SensorDataQoS().reliable(),
      std::bind(&LimoDiffDriver::cmdCallback, this, std::placeholders::_1));
}

void LimoDiffDriver::cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  cmd_vel_msg_.linear = msg->linear;
  cmd_vel_msg_.angular = msg->angular;
}

void LimoDiffDriver::step() {
  auto forward_speed = cmd_vel_msg_.linear.x;
  auto angular_speed = cmd_vel_msg_.angular.z;

  auto v_left =
      (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) /
      WHEEL_RADIUS;
  auto v_right =
      (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) /
      WHEEL_RADIUS;

  wb_motor_set_velocity(fl_motor_, v_left);
  wb_motor_set_velocity(rl_motor_, v_left);
  wb_motor_set_velocity(rr_motor_, v_right);
  wb_motor_set_velocity(fr_motor_, v_right);
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(limo_webot_simulation::LimoDiffDriver, webots_ros2_driver::PluginInterface)
