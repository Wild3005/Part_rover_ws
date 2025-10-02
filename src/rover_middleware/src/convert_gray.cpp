#include <iostream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

using namespace std::chrono_literals;

class convert_gray : public rclcpp::Node {
public:
  convert_gray() : Node("convert_gray") {
    // Create subscription (camera raw image)
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image", rclcpp::SensorDataQoS(),
      std::bind(&convert_gray::image_callback, this, std::placeholders::_1));

    // Create publisher (grayscale image)
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "camera_image_gray", rclcpp::SensorDataQoS());

    RCLCPP_INFO(this->get_logger(), "convert_gray node started.");
  }

private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    try {
      // Convert ROS image message to OpenCV image
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      cv::Mat img = cv_ptr->image;

      // Convert to grayscale
      cv::Mat gray_img;
      cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY);

      // Convert back to ROS message and publish
      auto gray_msg = cv_bridge::CvImage(msg->header, "mono8", gray_img).toImageMsg();
      image_pub_->publish(*gray_msg);

    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<convert_gray>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
