#include <iostream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

#include "image_transport/image_transport.hpp"

using namespace std::chrono_literals;

class convert_gray : public rclcpp::Node {
public:
  convert_gray() : Node("convert_gray") {
    RCLCPP_INFO(this->get_logger(), "convert_gray node constructed.");
  }

  static std::shared_ptr<convert_gray> create() {
    auto node = std::shared_ptr<convert_gray>(new convert_gray());
    node->init();
    return node;
  }

  void init() {
    // Gunakan rclcpp::Node::shared_from_this()
    image_transport_ = std::make_shared<image_transport::ImageTransport>(this->shared_from_this());

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image", rclcpp::SensorDataQoS(),
      std::bind(&convert_gray::image_callback, this, std::placeholders::_1));

    image_pub_ = image_transport_->advertise("camera_image_gray", 1);

    RCLCPP_INFO(this->get_logger(), "convert_gray node initialized.");
  }

private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    try {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      cv::Mat img = cv_ptr->image;

      cv::Mat gray_img;
      cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY);

      auto gray_msg = cv_bridge::CvImage(msg->header, "mono8", gray_img).toImageMsg();

      image_pub_.publish(gray_msg);

    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  image_transport::Publisher image_pub_;
  std::shared_ptr<image_transport::ImageTransport> image_transport_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = convert_gray::create();  // ✅ factory function dipanggil
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
