#include "rclcpp/rclcpp.hpp"
#include "livox_mid360_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LivoxMid360Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
