#include "rclcpp/rclcpp.hpp"
#include "livox_sdk2_driver/livox_mid360.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LivoxMid360Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
