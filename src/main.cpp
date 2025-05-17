#include "rclcpp/rclcpp.hpp"
#include "livox_sdk2_driver/livox_mid360.hpp"

int main(int argc, char ** argv)
{
  RCLCPP_INFO(rclcpp::get_logger("livox_mid360"), "Starting LivoxMid360Node main");
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("livox_mid360"), "Starting LivoxMid360Node");
  auto node = std::make_shared<LivoxMid360Node>();
  RCLCPP_INFO(rclcpp::get_logger("livox_mid360"), "LivoxMid360Node created, now spinning");
  rclcpp::spin(node);
  RCLCPP_INFO(rclcpp::get_logger("livox_mid360"), "LivoxMid360Node finished spinning");
  rclcpp::shutdown();
  return 0;
}
