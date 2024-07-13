#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

class DistancePublisher : public rclcpp::Node {
public:
    DistancePublisher() : Node("distance_publisher") {
        publisher_dist_right_ = this->create_publisher<std_msgs::msg::Float32>("dist_right", 10);
        publisher_dist_left_ = this->create_publisher<std_msgs::msg::Float32>("dist_left", 10);
    }

    void publish_distances(float dist_right, float dist_left) {
        auto msg_right = std_msgs::msg::Float32();
        auto msg_left = std_msgs::msg::Float32();
        msg_right.data = dist_right;
        msg_left.data = dist_left;
        publisher_dist_right_->publish(msg_right);
        publisher_dist_left_->publish(msg_left);
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_dist_right_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_dist_left_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DistancePublisher>();

    while (true) {
        std::cout << "Enter distances (right left): ";
        float dist_right, dist_left;
        std::cin >> dist_right >> dist_left;
        node->publish_distances(dist_right, dist_left);
    }

    rclcpp::shutdown();
    return 0;
}