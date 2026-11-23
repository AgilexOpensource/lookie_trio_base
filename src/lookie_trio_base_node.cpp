#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "lookie_trio_base/lookie_trio_base.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lookie_trio_base::LookieTrioBase>();
  int code = 0;
  if (node->Start()) {
    rclcpp::spin(node);
    node->Stop();
  } else {
    code = 1;
  }
  node.reset();
  rclcpp::shutdown();
  return code;
}
