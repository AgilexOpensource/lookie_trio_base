#ifndef LOOKIE_TRIO_BASE__ODOMETRY_HPP_
#define LOOKIE_TRIO_BASE__ODOMETRY_HPP_

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace lookie_trio_base
{

// Signs (+1 or -1) of the forward speed, the lateral speed and the yaw rate between ROS and the base.
struct VelocitySigns
{
  int x = 1;
  int y = 1;
  int wz = 1;
};

inline VelocitySigns ParseVelocitySigns(const std::vector<int64_t> & values, const std::string & name)
{
  if (values.size() != 3) {
    throw std::invalid_argument(name + " must have 3 entries [x, y, wz]");
  }
  for (int64_t v : values) {
    if (v != 1 && v != -1) {
      throw std::invalid_argument(name + " entries must be +1 or -1");
    }
  }
  return {static_cast<int>(values[0]), static_cast<int>(values[1]), static_cast<int>(values[2])};
}

inline double ValidateFeedbackTimeout(double timeout_s)
{
  if (!std::isfinite(timeout_s) || timeout_s <= 0.0) {
    throw std::invalid_argument("feedback_timeout must be a positive number");
  }
  return timeout_s;
}

// The SDK keeps the last state after CAN goes silent; its receive stamp tells whether it is still updated.
template<typename TimePoint>
inline bool IsFeedbackFresh(TimePoint stamp, TimePoint now, double timeout_s)
{
  return std::chrono::duration<double>(now - stamp).count() <= timeout_s;
}

// Row-major 6x6 diagonal: x, y, z, roll, pitch, yaw. Planar terms are measured, the rest is unknown.
constexpr double kCovarianceUnknown = 1e6;

inline void FillCovariance(std::array<double, 36> & covariance, double planar, double yaw)
{
  covariance.fill(0.0);
  covariance[0] = covariance[7] = planar;
  covariance[14] = covariance[21] = covariance[28] = kCovarianceUnknown;
  covariance[35] = yaw;
}

constexpr double kPi = 3.14159265358979323846;

// Keeps the heading in [-pi, pi] so that long runs do not accumulate error.
inline double NormalizeAngle(double angle)
{
  return std::remainder(angle, 2.0 * kPi);
}

// A stalled timer must not turn into one large pose jump.
constexpr double kMaxStepSeconds = 0.2;

inline double ClampStepSeconds(double seconds)
{
  if (!std::isfinite(seconds) || seconds < 0.0) {
    return 0.0;
  }
  return std::min(seconds, kMaxStepSeconds);
}

struct OdometryStep
{
  double linear_speed;
  double lateral_speed;
  double angular_speed;
  double dx;
  double dy;
  double dtheta;
};

// The body velocity (forward, lateral) is rotated into the odometry frame by the yaw; a differential base has no lateral speed.
inline OdometryStep ComputeOdometryStep(
  double linear_speed, double lateral_speed, double angular_speed, double yaw, double dt,
  const VelocitySigns & signs = {})
{
  linear_speed *= signs.x;
  lateral_speed *= signs.y;
  angular_speed *= signs.wz;
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return {linear_speed, lateral_speed, angular_speed,
    (linear_speed * c - lateral_speed * s) * dt,
    (linear_speed * s + lateral_speed * c) * dt,
    angular_speed * dt};
}

}  // namespace lookie_trio_base

#endif  // LOOKIE_TRIO_BASE__ODOMETRY_HPP_
