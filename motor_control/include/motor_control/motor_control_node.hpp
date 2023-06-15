#ifndef MOTOR_CONTROL_NODE_HPP
#define MOTOR_CONTROL_NODE_HPP

#include <vector>
#include <chrono>

#include "beagle_msgs/action/motor_control.hpp"
#include "beagle_msgs/srv/robot_motor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class MotorControl : public rclcpp::Node {
public:
  using robot_motor = beagle_msgs::srv::RobotMotor;
  using motor_control = beagle_msgs::action::MotorControl;
  using goal_handle_motor_control = rclcpp_action::ServerGoalHandle<motor_control>;
  MotorControl();
  ~MotorControl();

private:
  rclcpp::Node::SharedPtr node;
  float encoder_l,encoder_r;
  rclcpp::Client<robot_motor>::SharedPtr robot_motor_client_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const motor_control::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<goal_handle_motor_control> goal_handle);
  void motor_controling(const std::shared_ptr<goal_handle_motor_control> goal_handle);

  rclcpp_action::Server<motor_control>::SharedPtr
    motor_control_action_server_;
};

#endif
