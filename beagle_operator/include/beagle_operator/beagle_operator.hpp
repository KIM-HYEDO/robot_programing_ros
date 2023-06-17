#ifndef BEAGLE_OPERATOR_HPP_
#define BEAGLE_OPERATOR_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <random>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

// #include "beagle_msgs/msg/forward.hpp"
#include "beagle_msgs/srv/connect_checker.hpp"

class BeagleOperator : public rclcpp::Node {
public:

    explicit BeagleOperator(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
    virtual ~BeagleOperator();

    // Node를 주기적으로 실행시켜 줄 timer  rclcpp::TimerBase
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;


    // rclcpp::Publisher<beagle_msgs::msg::Forward>::SharedPtr    pub_Forward;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_Forward;

    //srv - Client
    rclcpp::Client<beagle_msgs::srv::ConnectChecker>::SharedPtr clinet_connect_checker;


    // Methods
    void ConnectChecker();

};

#endif
