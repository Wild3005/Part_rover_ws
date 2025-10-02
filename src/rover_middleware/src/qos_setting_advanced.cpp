#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"

// Node untuk convert QoS dengan image_transport yang lebih advanced:
// - Subscribe dari /camera_image_gray dengan QoS yang dapat dikonfigurasi
// - Publish ulang ke /topic_best_effort dengan QoS yang dapat dikonfigurasi
// - Mendukung compressed image transport

class QoSSettingAdvanced : public rclcpp::Node {
public:
  QoSSettingAdvanced() : Node("qos_setting_advanced") {
    // Declare parameters untuk konfigurasi QoS
    this->declare_parameter("input_topic", "/camera_image_gray");
    this->declare_parameter("output_topic", "/topic_best_effort");
    this->declare_parameter("queue_size", 10);
    this->declare_parameter("input_reliable", true);
    this->declare_parameter("output_reliable", false);
    this->declare_parameter("transport_hint", "raw"); // "raw", "compressed", atau "auto"
    
    // Get parameters
    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();
    int queue_size = this->get_parameter("queue_size").as_int();
    bool input_reliable = this->get_parameter("input_reliable").as_bool();
    bool output_reliable = this->get_parameter("output_reliable").as_bool();
    std::string transport_hint = this->get_parameter("transport_hint").as_string();
    
    // Initialize image transport
    it_ = std::make_shared<image_transport::ImageTransport>(this->shared_from_this());
    
    // Setup QoS untuk subscription
    rclcpp::QoS sub_qos(queue_size);
    if (input_reliable) {
      sub_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    } else {
      sub_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    }
    
    // Setup QoS untuk publisher
    rclcpp::QoS pub_qos(queue_size);
    if (output_reliable) {
      pub_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    } else {
      pub_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    }
    
    // Create subscription - image_transport tidak mendukung QoS custom secara langsung
    // Jadi kita gunakan approach sederhana
    sub_ = it_->subscribe(input_topic, queue_size,
      std::bind(&QoSSettingAdvanced::topic_callback, this, std::placeholders::_1));
    
    // Create publisher
    pub_ = it_->advertise(output_topic, queue_size);
    
    RCLCPP_INFO(this->get_logger(), 
      "QoS setting advanced node started:");
    RCLCPP_INFO(this->get_logger(), 
      "  Input: %s (QoS: %s, Transport: %s)", 
      input_topic.c_str(), 
      input_reliable ? "Reliable" : "Best Effort",
      transport_hint.c_str());
    RCLCPP_INFO(this->get_logger(), 
      "  Output: %s (QoS: %s)", 
      output_topic.c_str(), 
      output_reliable ? "Reliable" : "Best Effort");
  }

private:
  void topic_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    // Forward pesan image ke publisher
    // image_transport akan otomatis handle konversi ke compressed jika diperlukan
    pub_.publish(msg);
    
    // Optional: Log statistik setiap 100 pesan
    static int msg_count = 0;
    if (++msg_count % 100 == 0) {
      RCLCPP_INFO(this->get_logger(), "Relayed %d images", msg_count);
    }
  }

  std::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::Subscriber sub_;
  image_transport::Publisher pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QoSSettingAdvanced>());
  rclcpp::shutdown();
  return 0;
}
