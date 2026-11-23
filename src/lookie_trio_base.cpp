#include "lookie_trio_base/lookie_trio_base.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <vector>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace lookie_trio_base
{
namespace
{
constexpr double kPublishRateHz = 50.0;
constexpr double kReportPeriod = 1.0;  // battery and diagnostics, s
constexpr int kConnectRetrySeconds = 1;
constexpr std::size_t kMotorCount = 3;
constexpr double kBatteryVoltageScale = 0.1;  // the SDK reports ten times the battery voltage of this base
constexpr double kRemoteAxisScale = 0.01;  // sticks and knob report -100 to 100
constexpr double kCovariancePlanar = 1e-2;  // measured pose and velocity
constexpr double kCovarianceYaw = 1e-2;
// One driver serves every wheel: only the first low-speed state frame is sent, with the motor temperature negated.
constexpr double kMotorTemperatureScale = -1.0;

rcl_interfaces::msg::ParameterDescriptor ReadOnly(const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor d;
  d.read_only = true;
  d.description = description;
  return d;
}

diagnostic_msgs::msg::KeyValue Value(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = key;
  kv.value = value;
  return kv;
}

std::string Number(double value)
{
  char text[32];
  std::snprintf(text, sizeof(text), "%.2f", value);
  return text;
}

std::string Hex(unsigned value)
{
  char text[16];
  std::snprintf(text, sizeof(text), "0x%02X", value);
  return text;
}

double SwitchAxis(uint8_t position)
{
  switch (position) {
    case RC_SWITCH_UP: return 1.0;
    case RC_SWITCH_DOWN: return -1.0;
    default: return 0.0;
  }
}

const char * ControlModeName(unsigned mode)
{
  switch (mode) {
    case CONTROL_MODE_STANDBY: return "standby";
    case CONTROL_MODE_CAN: return "CAN";
    case CONTROL_MODE_UART: return "UART";
    case CONTROL_MODE_RC: return "remote controller";
    default: return "unknown";
  }
}
}  // namespace

LookieTrioBase::LookieTrioBase(const rclcpp::NodeOptions & options)
: rclcpp::Node("lookie_trio_base", options)
{
  port_name_ = declare_parameter("port_name", "can_chassis", ReadOnly("CAN interface of the base"));
  connect_retries_ = std::max<int64_t>(
    declare_parameter("connect_retries", 5,
    ReadOnly("Attempts to open the bus and connect to the base, one per second, before giving up")), 1);
  odom_frame_ = declare_parameter("odom_frame", "odom", ReadOnly("Odometry frame id"));
  base_frame_ = declare_parameter("base_frame", "base_link", ReadOnly("Base frame id"));
  publish_odom_tf_ = declare_parameter(
    "publish_odom_tf", true,
    ReadOnly("Broadcast the odom_frame -> base_frame TF; disable when another source owns it"));
  cmd_signs_ = ParseVelocitySigns(
    declare_parameter("cmd_velocity_signs", std::vector<int64_t>{1, -1, 1},
    ReadOnly("Signs [x, y, wz] (+1 or -1) applied to cmd_vel before it is sent to the base")),
    "cmd_velocity_signs");
  odom_signs_ = ParseVelocitySigns(
    declare_parameter("odom_velocity_signs", std::vector<int64_t>{-1, 1, 1},
    ReadOnly("Signs [x, y, wz] (+1 or -1) applied to the base feedback")),
    "odom_velocity_signs");
  feedback_timeout_ = ValidateFeedbackTimeout(
    declare_parameter("feedback_timeout", 0.5,
    ReadOnly("Seconds without CAN motion state before odometry stops publishing")));

  RCLCPP_INFO(
    get_logger(), "port %s, frames %s -> %s (TF %s), cmd signs [%d, %d, %d], odom signs [%d, %d, %d], "
    "feedback timeout %.2f s", port_name_.c_str(), odom_frame_.c_str(), base_frame_.c_str(),
    publish_odom_tf_ ? "on" : "off", cmd_signs_.x, cmd_signs_.y, cmd_signs_.wz,
    odom_signs_.x, odom_signs_.y, odom_signs_.wz, feedback_timeout_);
}

bool LookieTrioBase::Connect()
{
  // The chassis model is fixed by this package, so the protocol does not need to
  // be sniffed on the bus. Sniffing used a ProtocolDetector, whose CAN callback
  // stays bound to the detector after it goes out of scope; a base that is not
  // answering now shows up as missing feedback instead, which Publish() reports.
  robot_ = std::make_shared<Robot>(ProtocolVersion::AGX_V2);
  if (!robot_->Connect(port_name_)) {
    RCLCPP_WARN(get_logger(), "cannot open CAN interface %s", port_name_.c_str());
    robot_.reset();
    return false;
  }
  robot_->EnableCommandedMode();
  return true;
}

bool LookieTrioBase::Start()
{
  for (int64_t attempt = 1; !Connect(); ++attempt) {
    if (attempt >= connect_retries_) {
      RCLCPP_ERROR(get_logger(), "giving up on %s after %ld attempts", port_name_.c_str(), attempt);
      return false;
    }
    RCLCPP_WARN(
      get_logger(), "retrying in %d s (attempt %ld of %ld)", kConnectRetrySeconds, attempt + 1,
      connect_retries_);
    rclcpp::sleep_for(std::chrono::seconds(kConnectRetrySeconds));
  }

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 50);
  battery_pub_ = create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 10);
  joy_pub_ = create_publisher<sensor_msgs::msg::Joy>("rc/joy", 10);
  diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 5, std::bind(&LookieTrioBase::OnCmdVel, this, std::placeholders::_1));
  front_light_srv_ = create_service<std_srvs::srv::SetBool>(
    "front_light",
    std::bind(&LookieTrioBase::OnFrontLight, this, std::placeholders::_1, std::placeholders::_2));
  if (publish_odom_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  last_time_ = now();
  last_report_time_ = last_time_ - rclcpp::Duration::from_seconds(kReportPeriod);
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / kPublishRateHz), std::bind(&LookieTrioBase::Publish, this));
  RCLCPP_INFO(get_logger(), "Lookie Trio base connected on %s", port_name_.c_str());
  return true;
}

void LookieTrioBase::OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  robot_->SetMotionCommand(
    cmd_signs_.x * msg->linear.x, cmd_signs_.wz * msg->angular.z, cmd_signs_.y * msg->linear.y);
}

void LookieTrioBase::OnFrontLight(
  const std_srvs::srv::SetBool::Request::SharedPtr request,
  std_srvs::srv::SetBool::Response::SharedPtr response)
{
  // The rear light is left off: only the front light is part of this interface.
  robot_->SetLightCommand(request->data ? CONST_ON : CONST_OFF, 0, CONST_OFF, 0);
  front_light_on_ = request->data;
  response->success = true;
  response->message = front_light_on_ ? "front light on" : "front light off";
  RCLCPP_INFO(get_logger(), "front light %s", front_light_on_ ? "on" : "off");
}

void LookieTrioBase::Stop()
{
  if (!robot_) {
    return;
  }
  timer_.reset();
  robot_->SetMotionCommand(0.0, 0.0, 0.0);
  RCLCPP_INFO(get_logger(), "base stopped");
}

void LookieTrioBase::Publish()
{
  const rclcpp::Time stamp = now();
  const double dt = ClampStepSeconds((stamp - last_time_).seconds());
  last_time_ = stamp;

  // The SDK keeps the last state after CAN goes silent, so only fresh feedback is published.
  const auto state = robot_->GetRobotState();
  const bool fresh = IsFeedbackFresh(state.time_stamp, westonrobot::SdkClock::now(), feedback_timeout_);
  if (fresh != feedback_alive_) {
    feedback_alive_ = fresh;
    if (fresh) {
      RCLCPP_INFO(get_logger(), "base feedback %s", feedback_seen_ ? "recovered" : "received");
    } else {
      RCLCPP_WARN(get_logger(), "base feedback lost (no CAN state for %.2f s), odometry paused", feedback_timeout_);
    }
    feedback_seen_ = true;
  }
  if (fresh) {
    // A stale gap is skipped rather than integrated with the last cached velocity.
    PublishOdometry(
      state.motion_state.linear_velocity, state.motion_state.lateral_velocity, state.motion_state.angular_velocity, dt);

    sensor_msgs::msg::Joy joy;
    joy.header.stamp = stamp;
    joy.axes = {
      static_cast<float>(state.rc_state.stick_left_h * kRemoteAxisScale),
      static_cast<float>(state.rc_state.stick_left_v * kRemoteAxisScale),
      static_cast<float>(state.rc_state.stick_right_h * kRemoteAxisScale),
      static_cast<float>(state.rc_state.stick_right_v * kRemoteAxisScale),
      static_cast<float>(state.rc_state.var_a * kRemoteAxisScale),
      static_cast<float>(SwitchAxis(state.rc_state.swa)),
      static_cast<float>(SwitchAxis(state.rc_state.swb)),
      static_cast<float>(SwitchAxis(state.rc_state.swc)),
      static_cast<float>(SwitchAxis(state.rc_state.swd))};
    joy_pub_->publish(joy);
  }
  if ((stamp - last_report_time_).seconds() >= kReportPeriod) {
    last_report_time_ = stamp;
    PublishReports(stamp, fresh);
  }
}

void LookieTrioBase::PublishReports(const rclcpp::Time & stamp, bool fresh)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  const auto state = robot_->GetRobotState();
  const double voltage = state.system_state.battery_voltage * kBatteryVoltageScale;

  if (fresh) {
    sensor_msgs::msg::BatteryState battery;
    battery.header.stamp = stamp;
    battery.voltage = static_cast<float>(voltage);
    battery.temperature = std::numeric_limits<float>::quiet_NaN();
    battery.current = std::numeric_limits<float>::quiet_NaN();
    battery.charge = std::numeric_limits<float>::quiet_NaN();
    battery.capacity = std::numeric_limits<float>::quiet_NaN();
    battery.design_capacity = std::numeric_limits<float>::quiet_NaN();
    battery.percentage = std::numeric_limits<float>::quiet_NaN();
    battery.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    battery.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    battery.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
    battery.present = true;
    battery_pub_->publish(battery);
  }

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = stamp;
  const std::string prefix = std::string(get_fully_qualified_name()) + ": ";

  DiagnosticStatus base;
  base.name = prefix + "base";
  base.hardware_id = port_name_;
  if (!fresh) {
    base.level = DiagnosticStatus::STALE;
    base.message = "no CAN feedback";
  } else if (state.system_state.control_mode != CONTROL_MODE_CAN) {
    base.level = DiagnosticStatus::WARN;
    base.message = "not in CAN command mode, cmd_vel is ignored";
  } else {
    base.level = DiagnosticStatus::OK;
    base.message = "OK";
  }
  base.values = {
    Value("control_mode", ControlModeName(state.system_state.control_mode)),
    Value("vehicle_state", std::to_string(state.system_state.vehicle_state)),
    Value("error_code", Hex(state.system_state.error_code)),
    Value("battery_voltage", Number(voltage)),
    Value("front_light", front_light_on_ ? "on" : "off")};
  array.status.push_back(base);

  const auto actuator = robot_->GetActuatorState();
  for (std::size_t i = 0; i < kMotorCount; ++i) {
    const auto & motor = actuator.actuator_hs_state[i];
    const auto & driver = actuator.actuator_ls_state[0];
    DiagnosticStatus status;
    status.name = prefix + "motor " + std::to_string(i + 1);
    status.hardware_id = port_name_;
    status.level = fresh ? DiagnosticStatus::OK : DiagnosticStatus::STALE;
    status.message = fresh ? "OK" : "no CAN feedback";
    status.values = {
      Value("rpm", std::to_string(motor.rpm)),
      Value("current", Number(motor.current)),
      Value("pulse_count", std::to_string(motor.pulse_count)),
      Value("driver_voltage", Number(driver.driver_voltage)),
      Value("driver_temperature", Number(driver.driver_temp)),
      Value("motor_temperature", Number(driver.motor_temp * kMotorTemperatureScale)),
      Value("driver_state", Hex(driver.driver_state))};
    array.status.push_back(status);
  }
  diagnostics_pub_->publish(array);
}

void LookieTrioBase::PublishOdometry(
  double linear_speed, double lateral_speed, double angular_speed, double dt)
{
  const auto step = ComputeOdometryStep(linear_speed, lateral_speed, angular_speed, theta_, dt, odom_signs_);
  x_ += step.dx;
  y_ += step.dy;
  theta_ = NormalizeAngle(theta_ + step.dtheta);

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, theta_);
  const auto orientation = tf2::toMsg(q);

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = last_time_;
  tf.header.frame_id = odom_frame_;
  tf.child_frame_id = base_frame_;
  tf.transform.translation.x = x_;
  tf.transform.translation.y = y_;
  tf.transform.rotation = orientation;
  if (tf_broadcaster_) {
    tf_broadcaster_->sendTransform(tf);
  }

  nav_msgs::msg::Odometry odom;
  odom.header = tf.header;
  odom.child_frame_id = base_frame_;
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.orientation = orientation;
  odom.twist.twist.linear.x = step.linear_speed;
  odom.twist.twist.linear.y = step.lateral_speed;
  odom.twist.twist.angular.z = step.angular_speed;
  FillCovariance(odom.pose.covariance, kCovariancePlanar, kCovarianceYaw);
  FillCovariance(odom.twist.covariance, kCovariancePlanar, kCovarianceYaw);
  odom_pub_->publish(odom);
}

}  // namespace lookie_trio_base
