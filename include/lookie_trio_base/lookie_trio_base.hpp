#ifndef LOOKIE_TRIO_BASE__LOOKIE_TRIO_BASE_HPP_
#define LOOKIE_TRIO_BASE__LOOKIE_TRIO_BASE_HPP_

#include <cstdint>
#include <memory>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "lookie_trio_base/odometry.hpp"
#include "ugv_sdk/mobile_robot/scout_robot.hpp"

namespace lookie_trio_base
{

// Lookie Trio omnidirectional base (AgileX CAN protocol, three drive motors on one driver).
class LookieTrioBase : public rclcpp::Node
{
public:
  explicit LookieTrioBase(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Connects to the CAN bus and starts publishing; false when the base is absent.
  bool Start();

  // Sends a zero velocity so that the base does not keep the last command.
  void Stop();

private:
  using Robot = westonrobot::ScoutMiniOmniRobot;

  // Opens the CAN bus and enables commanded mode; false leaves the base untouched.
  bool Connect();

  void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg);
  // Keeps the front light on or off; the rear light is not driven.
  void OnFrontLight(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response);

  void Publish();
  void PublishOdometry(double linear_speed, double lateral_speed, double angular_speed, double dt);
  void PublishReports(const rclcpp::Time & stamp, bool fresh);

  std::string port_name_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_odom_tf_ = true;
  VelocitySigns cmd_signs_;
  VelocitySigns odom_signs_;
  double feedback_timeout_ = 0.5;
  int64_t connect_retries_ = 5;

  std::shared_ptr<Robot> robot_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr front_light_srv_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time last_time_;
  rclcpp::Time last_report_time_;
  bool feedback_alive_ = false;
  bool feedback_seen_ = false;
  bool front_light_on_ = false;
  double x_ = 0.0;
  double y_ = 0.0;
  double theta_ = 0.0;
};

}  // namespace lookie_trio_base

#endif  // LOOKIE_TRIO_BASE__LOOKIE_TRIO_BASE_HPP_
