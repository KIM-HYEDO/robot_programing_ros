#include "../include/moter_control/moter_control_node.hpp"

using std::placeholders::_1;

MoterControl::MoterControl() : Node("moter_control") {
  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

  moter_control_server_ = this->create_service<moter_control>(
      "/moter_control",
      std::bind(&MoterControl::moter_control_callback, this,
                std::placeholders::_1, std::placeholders::_2));
  robot_moter_client_ =
      this->create_client<robot_moter>("/robot_moter");
}
MoterControl::~MoterControl() {}

void MoterControl::moter_control_callback(
    std::shared_ptr<moter_control::Request> request,
    std::shared_ptr<moter_control::Response> response) {
  RCLCPP_INFO(this->get_logger(), "hello");
  robot_moter::Request robot_moter_srv;
  switch (request->mode) {
  case 0: {

    break;
  }
  case 1: {
    break;
  }
  }
}
