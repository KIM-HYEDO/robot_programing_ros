#ifndef MOTOR_CONTROL_NODE_HPP
#define MOTOR_CONTROL_NODE_HPP

#include <chrono>
#include <vector>

#include "beagle_msgs/action/motor_control.hpp"
#include "beagle_msgs/srv/robot_motor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#define PI 3.141592

class MotorControl : public rclcpp::Node {
public:
  using robot_motor = beagle_msgs::srv::RobotMotor;
  using motor_control = beagle_msgs::action::MotorControl;
  using goal_handle_motor_control =
      rclcpp_action::ServerGoalHandle<motor_control>;
  MotorControl();
  ~MotorControl();
  void run();
private:
  const float WHEEL_BASE,WHEEL_RADIAN;
  const int TURN_PULSE;
  template <class T> T limit(T value, T limit = 100);
  rclcpp::Node::SharedPtr node;
  int motor_r_,motor_l_;
  int target_encoder_r_, target_encoder_l_;
  int encoder_r_, encoder_l_;
  rclcpp::Client<robot_motor>::SharedPtr robot_motor_client_;

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const motor_control::Goal> goal);
  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<goal_handle_motor_control> goal_handle);
  void motor_controlling(
      const std::shared_ptr<goal_handle_motor_control> goal_handle);
  void control();

  rclcpp_action::Server<motor_control>::SharedPtr motor_control_action_server_;
  int action_seq_;
  std::shared_ptr<goal_handle_motor_control> goal_handle_;
  std::chrono::system_clock::time_point start_;
};

#endif
