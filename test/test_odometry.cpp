#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "lookie_trio_base/odometry.hpp"

namespace
{
void Check(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

bool Near(double actual, double expected) {return std::abs(actual - expected) < 1e-12;}
}  // namespace

int main()
{
  using lookie_trio_base::ClampStepSeconds;
  using lookie_trio_base::ComputeOdometryStep;
  using lookie_trio_base::kPi;
  using lookie_trio_base::NormalizeAngle;
  using lookie_trio_base::FillCovariance;
  using lookie_trio_base::IsFeedbackFresh;
  using lookie_trio_base::ParseVelocitySigns;
  using lookie_trio_base::ValidateFeedbackTimeout;
  using lookie_trio_base::VelocitySigns;
  try {
    const auto parsed = ParseVelocitySigns({-1, 1, -1}, "signs");
    Check(parsed.x == -1 && parsed.y == 1 && parsed.wz == -1, "parsed signs");
    for (const std::vector<int64_t> & bad :
      {std::vector<int64_t>{1, 1}, std::vector<int64_t>{1, 1, 1, 1}, std::vector<int64_t>{1, 0, 1}})
    {
      bool rejected = false;
      try {
        ParseVelocitySigns(bad, "signs");
      } catch (const std::invalid_argument &) {
        rejected = true;
      }
      Check(rejected, "invalid signs accepted");
    }

    const double half_pi = std::acos(-1.0) / 2.0;
    const auto forward = ComputeOdometryStep(0.02, 0.0, 0.0, half_pi, 1.0);
    Check(Near(forward.dx, 0.0) && Near(forward.dy, 0.02), "forward speed follows the heading");
    const auto lateral = ComputeOdometryStep(0.0, 0.02, 0.0, half_pi, 1.0);
    Check(Near(lateral.dx, -0.02) && Near(lateral.dy, 0.0), "lateral speed is rotated by the heading");
    const auto signed_step = ComputeOdometryStep(0.01, 0.02, 0.03, 0.0, 0.5, VelocitySigns{-1, 1, -1});
    Check(
      Near(signed_step.linear_speed, -0.01) && Near(signed_step.lateral_speed, 0.02) &&
      Near(signed_step.angular_speed, -0.03) && Near(signed_step.dx, -0.005) &&
      Near(signed_step.dy, 0.01) && Near(signed_step.dtheta, -0.015),
      "signs apply per axis");

    std::array<double, 36> covariance;
    FillCovariance(covariance, 0.01, 0.02);
    Check(Near(covariance[0], 0.01) && Near(covariance[7], 0.01), "planar covariance");
    Check(Near(covariance[35], 0.02), "yaw covariance");
    Check(covariance[14] > 1e5 && covariance[21] > 1e5 && covariance[28] > 1e5, "unknown covariance");
    Check(Near(covariance[1], 0.0) && Near(covariance[8], 0.0), "off-diagonal stays zero");

    Check(Near(NormalizeAngle(0.0), 0.0), "zero angle stays zero");
    Check(Near(NormalizeAngle(2.5 * kPi), 0.5 * kPi), "angle above pi wraps down");
    Check(Near(NormalizeAngle(-1.5 * kPi), 0.5 * kPi), "negative angle wraps up");
    Check(Near(NormalizeAngle(4.0 * kPi), 0.0), "full turns collapse");
    Check(std::abs(NormalizeAngle(7.0)) <= kPi, "wrapped angle stays in range");

    Check(Near(ClampStepSeconds(0.02), 0.02), "short step is kept");
    Check(Near(ClampStepSeconds(5.0), 0.2), "long step is capped");
    Check(Near(ClampStepSeconds(-1.0), 0.0), "negative step is dropped");
    Check(Near(ClampStepSeconds(std::nan("")), 0.0), "nan step is dropped");

    using Clock = std::chrono::steady_clock;
    const Clock::time_point now{std::chrono::seconds(100)};
    Check(IsFeedbackFresh(now - std::chrono::milliseconds(20), now, 0.5), "recent feedback is fresh");
    Check(!IsFeedbackFresh(now - std::chrono::milliseconds(600), now, 0.5), "old feedback is stale");
    for (double bad : {0.0, -1.0, std::nan("")}) {
      bool rejected = false;
      try {
        ValidateFeedbackTimeout(bad);
      } catch (const std::invalid_argument &) {
        rejected = true;
      }
      Check(rejected, "invalid feedback timeout accepted");
    }
  } catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }
  return 0;
}
