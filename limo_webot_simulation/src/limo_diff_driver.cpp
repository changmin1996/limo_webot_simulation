#include "limo_webot_simulation/limo_diff_driver.hpp"

#include "rclcpp/rclcpp.hpp"
#include <cstdio>
#include <functional>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/position_sensor.h>

#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <functional>

#define HALF_DISTANCE_BETWEEN_WHEELS 0.215
#define WHEEL_RADIUS 0.045

namespace limo_webot_simulation
{
void LimoDiffDriver::init(
  webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string>& parameters)
{
  node_ = node;

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

  fl_ps_ = wb_motor_get_position_sensor(fl_motor_);
  fr_ps_ = wb_motor_get_position_sensor(fr_motor_);
  rl_ps_ = wb_motor_get_position_sensor(rl_motor_);
  rr_ps_ = wb_motor_get_position_sensor(rr_motor_);

  const int timestep_ms = (int)wb_robot_get_basic_time_step();
  dt_ = timestep_ms / 1000.0;

  wb_position_sensor_enable(fl_ps_, timestep_ms);
  wb_position_sensor_enable(fr_ps_, timestep_ms);
  wb_position_sensor_enable(rl_ps_, timestep_ms);
  wb_position_sensor_enable(rr_ps_, timestep_ms);

  cmd_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::SensorDataQoS().reliable(),
      std::bind(&LimoDiffDriver::cmdCallback, this, std::placeholders::_1));

  odom_pub_ = node->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);

  odom_initialized_ = false;
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

  const double fl = wb_position_sensor_get_value(fl_ps_);
  const double rl = wb_position_sensor_get_value(rl_ps_);
  const double fr = wb_position_sensor_get_value(fr_ps_);
  const double rr = wb_position_sensor_get_value(rr_ps_);

  const double left_pos  = 0.5 * (fl + rl);   // rad
  const double right_pos = 0.5 * (fr + rr);   // rad

  if (!odom_initialized_) {
    prev_left_pos_ = left_pos;
    prev_right_pos_ = right_pos;
    odom_initialized_ = true;
    return;
  }

  const double dphi_l = left_pos  - prev_left_pos_;   // rad
  const double dphi_r = right_pos - prev_right_pos_;  // rad
  prev_left_pos_ = left_pos;
  prev_right_pos_ = right_pos;

  const double dl = dphi_l * wheel_radius_;
  const double dr = dphi_r * wheel_radius_;

  const double ds = 0.5 * (dr + dl);
  const double dtheta = (dr - dl) / (2.0 * half_track_);  // track = 2*half_track

  const double yaw_mid = yaw_ + 0.5 * dtheta;
  x_ += ds * std::cos(yaw_mid);
  y_ += ds * std::sin(yaw_mid);
  yaw_ += dtheta;

  const double v = (dt_ > 0.0) ? (ds / dt_) : 0.0;
  const double w = (dt_ > 0.0) ? (dtheta / dt_) : 0.0;

  const rclcpp::Time stamp = node_->get_clock()->now();

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";

  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw_);
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();

  odom.twist.twist.linear.x = v;
  odom.twist.twist.angular.z = w;

  odom_pub_->publish(odom);
  if (publish_tf_) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf);
  }
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(limo_webot_simulation::LimoDiffDriver, webots_ros2_driver::PluginInterface)
