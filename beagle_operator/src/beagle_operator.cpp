#include "beagle_operator/beagle_operator.hpp"

using namespace std::chrono_literals;

BeagleOperator::BeagleOperator(const rclcpp::NodeOptions & node_options)
: Node("beagle_operator", node_options)
{
    // Node 생성자에서 수행할 작업을 추가하세요.

    //Publisherac
    pub_Forward = this->create_publisher<beagle_msgs::msg::Forward>("toBeagle", 10);

    //Subscriber
    sub_Forward = this->create_subscription<std_msgs::msg::Bool>(
    "toOperator",
    10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) -> void
    {
        if(msg.get()->data==true)
        {
            ConnectChecker();
        }
    }
    );

    //srv-Client
    clinet_connect_checker = this->create_client<beagle_msgs::srv::ConnectChecker>("connect_checker");
}

BeagleOperator::~BeagleOperator()
{
    // Node 소멸자에서 수행할 작업을 추가하세요.
}

void BeagleOperator::ConnectChecker()
{
    auto request = std::make_shared<beagle_msgs::srv::ConnectChecker::Request>();

    request->try_connect = true;

    using ServiceResponseFuture = rclcpp::Client<beagle_msgs::srv::ConnectChecker>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future) {
      auto response = future.get();
      if(response->connected)
        RCLCPP_INFO(this->get_logger(), "Connected!");
    return;
    };

    auto future_result =
        clinet_connect_checker->async_send_request(request,response_received_callback);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto beagle = std::make_shared<BeagleOperator>(rclcpp::NodeOptions());

    rclcpp::spin(beagle);

    rclcpp::shutdown();

    return 0;
}
