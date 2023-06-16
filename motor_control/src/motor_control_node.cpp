#include "../include/motor_control/motor_control_node.hpp"

using namespace std::chrono_literals;

MotorControl::MotorControl()
    : Node("motor_control"), encoder_l_(0), encoder_r_(0), node(this),
      action_seq_(0), motor_l_(0), motor_r_(0) {
  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

  robot_motor_client_ = this->create_client<robot_motor>("/robot_motor");
  while (!robot_motor_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Interrupted while waiting for the service.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
  }

  using namespace std::placeholders;
  motor_control_action_server_ = rclcpp_action::create_server<motor_control>(
      this->get_node_base_interface(), this->get_node_clock_interface(),
      this->get_node_logging_interface(), this->get_node_waitables_interface(),
      "/motor_control", std::bind(&MotorControl::handle_goal, this, _1, _2),
      std::bind(&MotorControl::handle_cancel, this, _1),
      std::bind(&MotorControl::motor_controling, this, _1));
}
MotorControl::~MotorControl() {}

rclcpp_action::GoalResponse
MotorControl::handle_goal(const rclcpp_action::GoalUUID &uuid,
                          std::shared_ptr<const motor_control::Goal> goal) {
  RCLCPP_WARN(this->get_logger(), "new goal");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::CancelResponse MotorControl::handle_cancel(
    const std::shared_ptr<goal_handle_motor_control> goal_handle) {
  action_seq_ = 0;
  // goal_handle_.reset();
  RCLCPP_WARN(this->get_logger(), "cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}
void MotorControl::motor_controling(
    const std::shared_ptr<goal_handle_motor_control> goal_handle) {
  RCLCPP_WARN(this->get_logger(), "start goal");
  goal_handle_ = goal_handle;
  start_ = std::chrono::system_clock::now();
  action_seq_ = 1;
}
void MotorControl::run() {
  switch (action_seq_) {
  case 1: {
    {
      auto robot_req = std::make_shared<robot_motor::Request>();
      robot_req->motor_l = motor_l_;
      robot_req->motor_r = motor_r_;
      auto result = robot_motor_client_->async_send_request(robot_req);
      // Wait for the result.
      if (rclcpp::spin_until_future_complete(node, result) ==
          rclcpp::FutureReturnCode::SUCCESS) {
        encoder_l_ = result.get()->encoder_l;
        encoder_r_ = result.get()->encoder_r;
      } else {
        RCLCPP_WARN(this->get_logger(),
                    "robot motor service failed\ncheck beagle_robot node");
        return;
      }
    }
    int motor_mode = goal_handle_->get_goal()->mode;
    std::vector<float> motor_data = goal_handle_->get_goal()->data;
    switch (motor_mode) {
    case 2: {
      if (motor_data[0] > 0) {
        target_encoder_l_ = encoder_l_ + motor_data[1] * TURN_PULSE;
        target_encoder_r_ = encoder_r_ + motor_data[1] * TURN_PULSE;
      } else {
        target_encoder_l_ = encoder_l_ - motor_data[1] * TURN_PULSE;
        target_encoder_r_ = encoder_r_ - motor_data[1] * TURN_PULSE;
      }
      break;
    }
    case 4: {
      target_encoder_l_ =
          encoder_l_ + (((motor_data[1] / 360.0) * (WHEEL_BASE / 2.0)) /
                        (WHEEL_RADIAN * 2.0 * PI)) *
                           TURN_PULSE;
      target_encoder_r_ =
          encoder_r_ - (((motor_data[1] / 360.0) * (WHEEL_BASE / 2.0)) /
                        (WHEEL_RADIAN * 2.0 * PI)) *
                           TURN_PULSE;
      break;
    }
    }
    action_seq_ = 2;
    break;
  }
  case 2: {
    control();
    break;
  }
  case 3: {
    if (goal_handle_->get_goal()->mode) {
      auto robot_req = std::make_shared<robot_motor::Request>();
      robot_req->motor_l = 0;
      robot_req->motor_r = 0;
      auto result = robot_motor_client_->async_send_request(robot_req);
      // Wait for the result.
      if (rclcpp::spin_until_future_complete(node, result) ==
          rclcpp::FutureReturnCode::SUCCESS) {
        encoder_l_ = result.get()->encoder_l;
        encoder_r_ = result.get()->encoder_r;
      } else {
        RCLCPP_WARN(this->get_logger(),
                    "robot motor service failed\ncheck beagle_robot node");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "end");
    }

    std::chrono::milliseconds time =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now() - start_);
    auto result = std::make_shared<motor_control::Result>();
    result->end_time = time.count();
    goal_handle_->succeed(result);
    action_seq_ = 0;
    break;
  }
  }
}
void MotorControl::control() {

  int motor_mode = goal_handle_->get_goal()->mode;
  std::vector<float> motor_data = goal_handle_->get_goal()->data;
  auto robot_req = std::make_shared<robot_motor::Request>();
  switch (motor_mode) {
  case 0: {
    robot_req->motor_l = limit(motor_data[0]);
    robot_req->motor_r = limit(motor_data[1]);
    break;
  }
  case 1: {
    robot_req->motor_l = limit(motor_data[0]);
    robot_req->motor_r = limit(motor_data[1]);
    std::chrono::milliseconds time =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now() - start_);
    action_seq_ = (time.count() / 1000.0 > motor_data[2]) ? 3 : 2;
    break;
  }
  case 2: {
    robot_req->motor_l =
        limit(target_encoder_l_ - encoder_l_) * (limit(motor_data[0]) / 100.0);
    robot_req->motor_r =
        limit(target_encoder_r_ - encoder_r_) * (limit(motor_data[0]) / 100.0);
    action_seq_ = (std::abs(target_encoder_l_ - encoder_l_) +
                       std::abs(target_encoder_r_ - encoder_r_) <
                   TURN_PULSE / 100)
                      ? 3
                      : 2;

    break;
  }
  case 3: {
    robot_req->motor_l = limit(motor_data[0]);
    robot_req->motor_r = limit(motor_data[0]);
    std::chrono::milliseconds time =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now() - start_);
    action_seq_ = (time.count() / 1000.0 > motor_data[2]) ? 3 : 2;
    break;
  }
  case 4: {
    robot_req->motor_l =
        limit(target_encoder_l_ - encoder_l_) * (limit(motor_data[0]) / 100.0);
    robot_req->motor_r =
        limit(target_encoder_r_ - encoder_r_) * (limit(motor_data[0]) / 100.0);
    action_seq_ = (std::abs(target_encoder_l_ - encoder_l_) +
                       std::abs(target_encoder_r_ - encoder_r_) <
                   TURN_PULSE / 100)
                      ? 3
                      : 2;
    break;
  }
  }
  motor_l_ = robot_req->motor_l;
  motor_r_ = robot_req->motor_r;
  auto result = robot_motor_client_->async_send_request(robot_req);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    encoder_l_ = result.get()->encoder_l;
    encoder_r_ = result.get()->encoder_r;
  } else {
    RCLCPP_WARN(this->get_logger(),
                "robot motor service failed\ncheck beagle_robot node");
    return;
  }
  auto feedback = std::make_shared<motor_control::Feedback>();
  feedback->encoder_l = encoder_l_;
  feedback->encoder_r = encoder_r_;
  std::chrono::milliseconds time =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now() - start_);
  feedback->time = time.count();
  goal_handle_->publish_feedback(feedback);
}

template <class T> T MotorControl::limit(T value, T limit) {
  if (value > limit) {
    return limit;
  } else if (value < -limit) {
    return -limit;
  } else {
    return value;
  }
}
