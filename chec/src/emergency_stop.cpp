#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

class EmergencyStopServiceNode : public rclcpp::Node {
public:
  EmergencyStopServiceNode() : Node("emergency_stop_service_node") {
    // Create a subscriber to the emergency stop topic
    subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "emergency_stop", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
          // Print emergency stop message
          RCLCPP_INFO(this->get_logger(), "Emergency stop initiated!");
        }else {
      RCLCPP_INFO(this->get_logger(), "Emergency stop cleared.");
        }
      });
  }

private:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EmergencyStopServiceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}