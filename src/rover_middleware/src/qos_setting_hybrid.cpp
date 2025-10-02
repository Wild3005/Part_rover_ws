#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "image_transport/image_transport.hpp"

// Node untuk convert QoS dengan support untuk compressed dan custom QoS:
// - Bisa handle raw image dengan custom QoS
// - Bisa handle compressed image dengan custom QoS
// - Subscribe dan publish dengan QoS yang dapat dikonfigurasi

class QoSSettingHybrid : public rclcpp::Node {
public:
  QoSSettingHybrid() : Node("qos_setting_hybrid") {
    // Declare parameters untuk konfigurasi QoS
    this->declare_parameter("input_topic", "/camera_image_gray");
    this->declare_parameter("output_topic", "/topic_best_effort");
    this->declare_parameter("queue_size", 10);
    this->declare_parameter("input_reliable", true);
    this->declare_parameter("output_reliable", false);
    this->declare_parameter("use_compressed", true); // true untuk compressed, false untuk raw
    
    // Get parameters
    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();
    int queue_size = this->get_parameter("queue_size").as_int();
    bool input_reliable = this->get_parameter("input_reliable").as_bool();
    bool output_reliable = this->get_parameter("output_reliable").as_bool();
    bool use_compressed = this->get_parameter("use_compressed").as_bool();
    
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
    
    // Store settings
    use_compressed_ = use_compressed;
    
    if (use_compressed_) {
      // Use compressed image topics dengan custom QoS
      compressed_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        input_topic + "/compressed", sub_qos,
        std::bind(&QoSSettingHybrid::compressed_callback, this, std::placeholders::_1));
      
      compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        output_topic + "/compressed", pub_qos);
        
      RCLCPP_INFO(this->get_logger(), "Using COMPRESSED image transport");
    } else {
      // Use raw image topics dengan custom QoS
      raw_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        input_topic, sub_qos,
        std::bind(&QoSSettingHybrid::raw_callback, this, std::placeholders::_1));
      
      raw_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        output_topic, pub_qos);
        
      RCLCPP_INFO(this->get_logger(), "Using RAW image transport");
    }
    
    RCLCPP_INFO(this->get_logger(), 
      "QoS setting hybrid node started:");
    RCLCPP_INFO(this->get_logger(), 
      "  Input: %s (QoS: %s)", 
      input_topic.c_str(), 
      input_reliable ? "Reliable" : "Best Effort");
    RCLCPP_INFO(this->get_logger(), 
      "  Output: %s (QoS: %s)", 
      output_topic.c_str(), 
      output_reliable ? "Reliable" : "Best Effort");
  }

private:
  void raw_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    // Forward raw image message
    raw_pub_->publish(*msg);
    
    // Log statistik setiap 100 pesan
    static int msg_count = 0;
    if (++msg_count % 100 == 0) {
      RCLCPP_INFO(this->get_logger(), "Relayed %d raw images", msg_count);
    }
  }
  
  void compressed_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& msg) {
    // Forward compressed image message
    compressed_pub_->publish(*msg);
    
    // Log statistik setiap 100 pesan
    static int msg_count = 0;
    if (++msg_count % 100 == 0) {
      RCLCPP_INFO(this->get_logger(), "Relayed %d compressed images", msg_count);
    }
  }

  bool use_compressed_;
  
  // Raw image subscribers and publishers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_pub_;
  
  // Compressed image subscribers and publishers
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_sub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QoSSettingHybrid>());
  rclcpp::shutdown();
  return 0;
}
