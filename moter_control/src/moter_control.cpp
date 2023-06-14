#include <cstdio>
#include "../include/moter_control/moter_control_node.hpp"
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto motercontrol = std::make_shared<MoterControl>();

  rclcpp::spin(motercontrol);

  rclcpp::shutdown();

  return 0;
}
