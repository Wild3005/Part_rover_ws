#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/image.hpp"

// Node untuk convert QoS:
// - Subscribe dari /camera_image_gray dengan QoS Reliable
// - Publish ulang ke /topic_best_effort dengan QoS Best Effort

class QoSSetting : public rclcpp::Node {
public:
  QoSSetting() : Node("qos_setting") {
    // Subscription dengan QoS Reliable
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera_image_gray",
      rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
      std::bind(&QoSSetting::topic_callback, this, std::placeholders::_1));

    // Publisher dengan QoS Best Effort
    pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/topic_best_effort",
      rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());

    RCLCPP_INFO(this->get_logger(), "QoS setting node started (Image relay).");
  }

private:
  void topic_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    // Forward langsung pesan image ke publisher
    pub_->publish(*msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QoSSetting>());
  rclcpp::shutdown();
  return 0;
}
