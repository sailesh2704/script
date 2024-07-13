#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

class LimitSwitchPublisher : public rclcpp::Node {
public:
    LimitSwitchPublisher() : Node("limit_switch_publisher") {
        publisher_limit_switch_top_ = this->create_publisher<std_msgs::msg::Bool>("limit_switch_top", 10);
        publisher_limit_switch_bott_ = this->create_publisher<std_msgs::msg::Bool>("limit_switch_bott", 10);
    }

    void publish_limit_switch(bool top_state, bool bott_state) {
        auto msg_top = std_msgs::msg::Bool();
        msg_top.data = top_state;
        publisher_limit_switch_top_->publish(msg_top);

        auto msg_bott = std_msgs::msg::Bool();
        msg_bott.data = bott_state;
        publisher_limit_switch_bott_->publish(msg_bott);
    }

private:
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_limit_switch_top_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_limit_switch_bott_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LimitSwitchPublisher>();

    while (true) {
        std::cout << "Enter top and bottom limit switch states (0 0 or 1 1): ";
        int top_state, bott_state;
        std::cin >> top_state >> bott_state;
        node->publish_limit_switch(top_state!= 0, bott_state!= 0);
    }

    rclcpp::shutdown();
    return 0;
}