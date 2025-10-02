#include <iostream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

using namespace std::chrono_literals;

class OpenCam : public rclcpp::Node {
public:
  OpenCam() : Node("open_cam") {
    // Publisher untuk image
    pub_image_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/image", rclcpp::SensorDataQoS());

    // Timer untuk looping capture
    timer_ = this->create_wall_timer(
      33ms, std::bind(&OpenCam::timer_callback, this)); // ~30 FPS

    // Buka kamera (default: 0)
    cap_.open(0);
    // cap_.open(path_cam, cv::CAP_V4L);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open camera!");
    } else {
      RCLCPP_INFO(this->get_logger(), "Camera opened successfully.");
    }
  }

private:
  void timer_callback() {
    if (!cap_.isOpened()) {
      return;
    }

    cv::Mat frame;
    cap_ >> frame;
    if (frame.empty()) {
      RCLCPP_WARN(this->get_logger(), "Empty frame captured!");
      return;
    }

    // Convert OpenCV -> ROS2 message
    std_msgs::msg::Header header;
    header.stamp = this->now();
    auto msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();

    pub_image_->publish(*msg);
  }

  cv::VideoCapture cap_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string path_cam = "/dev/v4l/by-id/usb-GENERAL_XVV-6320S_JH1706_20211203_v004-video-index0";
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OpenCam>());
  rclcpp::shutdown();
  return 0;
}
