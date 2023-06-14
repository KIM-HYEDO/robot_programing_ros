#ifndef MOTER_CONTROL_NODE_HPP
#define MOTER_CONTROL_NODE_HPP

#include "beagle_msgs/srv/moter_control.hpp"
#include "beagle_msgs/srv/robot_moter.hpp"
#include "rclcpp/rclcpp.hpp"
class MoterControl : public rclcpp::Node {
public:
  using moter_control = beagle_msgs::srv::MoterControl;
  using robot_moter = beagle_msgs::srv::RobotMoter;
  MoterControl();
  ~MoterControl();

private:
  rclcpp::Service<moter_control>::SharedPtr moter_control_server_;
  void moter_control_callback(std::shared_ptr<moter_control::Request> request,
                         std::shared_ptr<moter_control::Response> response);
  rclcpp::Client<robot_moter>::SharedPtr robot_moter_client_;
};

#endif
