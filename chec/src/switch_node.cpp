#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

class LimitSwitchNode : public rclcpp::Node {
public:
  LimitSwitchNode() : Node("limit_switch_node"), previous_emergency_stop_(false) {
    // Create subscribers to the limit_switch_top and limit_switch_bott topics
    subscription_top_ = this->create_subscription<std_msgs::msg::Bool>(
      "limit_switch_top", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        top_switch_state_ = msg->data;
        checkSwitches();
      });

    subscription_bott_ = this->create_subscription<std_msgs::msg::Bool>(
      "limit_switch_bott", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        bott_switch_state_ = msg->data;
        checkSwitches();
      });

    // Create a publisher to the emergency stop topic
    emergency_stop_publisher_ = this->create_publisher<std_msgs::msg::Bool>("emergency_stop", 10);
  }

private:
  void checkSwitches() {
    bool emergency_stop = !top_switch_state_ || !bott_switch_state_;
    if (emergency_stop && !previous_emergency_stop_) {
      auto msg = std_msgs::msg::Bool();
      msg.data = true;
      emergency_stop_publisher_->publish(msg);
      previous_emergency_stop_ = true;
    } else if (!emergency_stop && previous_emergency_stop_) {
      previous_emergency_stop_ = false;
    }
  }

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_top_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_bott_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_stop_publisher_;
  bool top_switch_state_ = true;
  bool bott_switch_state_ = true;
  bool previous_emergency_stop_; // Declare and initialize the variable here
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LimitSwitchNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}