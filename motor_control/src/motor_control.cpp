#include <cstdio>
#include "../include/motor_control/motor_control_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto motorcontrol = std::make_shared<MotorControl>();

  rclcpp::spin(motorcontrol);

  rclcpp::shutdown();

  return 0;
}
