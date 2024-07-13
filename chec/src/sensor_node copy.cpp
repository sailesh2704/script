#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>

class UltrasonicSensorNode : public rclcpp::Node {
public:
  UltrasonicSensorNode() : Node("ultrasonic_sensor_node") {
    // Create subscribers to the dist_left and dist_right topics
    subscription_left_ = this->create_subscription<std_msgs::msg::Float32>(
      "dist_left", 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        left_distance_ = msg->data;
        checkLimits();
      });

    subscription_right_ = this->create_subscription<std_msgs::msg::Float32>(
      "dist_right", 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        right_distance_ = msg->data;
        checkLimits();
      });

    // Create a publisher to the emergency stop topic
    emergency_stop_publisher_ = this->create_publisher<std_msgs::msg::Bool>("emergency_stop", 10);
    emergency_stop_triggered_ = false;
  }

private:
  void checkLimits() {
    // Check if either distance is below the limit
    bool emergency_stop = (left_distance_ < 10.0 || right_distance_ < 10.0);
    if (emergency_stop &&!emergency_stop_triggered_) {
      // Publish a message to the emergency stop topic
      auto msg = std_msgs::msg::Bool();
      msg.data = true;
      emergency_stop_publisher_->publish(msg);
      emergency_stop_triggered_ = true;
    } else if (!emergency_stop && emergency_stop_triggered_) {
      // Publish a message to the emergency stop topic
      auto msg = std_msgs::msg::Bool();
      msg.data = false;
      emergency_stop_publisher_->publish(msg);
      emergency_stop_triggered_ = false;
    }
  }
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_left_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_right_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_stop_publisher_;
  float left_distance_ = 0.0;
  float right_distance_ = 0.0;
  bool emergency_stop_triggered_ = false;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UltrasonicSensorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}