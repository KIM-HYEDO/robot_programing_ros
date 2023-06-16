#include "../include/motor_control/motor_control_node.hpp"
#include <cstdio>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto motorcontrol = std::make_shared<MotorControl>();

  rclcpp::Rate loop(100);
  while (rclcpp::ok()) {
    rclcpp::spin_some(motorcontrol);
    motorcontrol->run();
    loop.sleep();
  }

  rclcpp::shutdown();

  return 0;
}
