#include <iostream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/char.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

using namespace std::chrono_literals;

class arm_pov : public rclcpp::Node {
public:
    arm_pov() : Node("arm_pov") {
        this->declare_parameter("qos_state_main", false);
        this->declare_parameter("qos_state_mini_arm", false);
        this->declare_parameter("qos_state_screenshot", false);

        this->declare_parameter("topic_main", "/image_main");
        this->declare_parameter("topic_mini_arm", "/image_mini_arm");
        this->declare_parameter("topic_screenshot", "/image_screenshot");

        this->declare_parameter("pub_main_compressed_image", false);
        this->declare_parameter("pub_mini_arm_compressed_image", false);
        this->declare_parameter("pub_screenshot_compressed_image", false);

        this->declare_parameter("path_cam_main", "/dev/v4l/by-id/usb-BC-231018-A_XWF_1080P_PC_Camera-video-index0");
        this->declare_parameter("path_cam_mini_arm", "/dev/v4l/by-id/usb-GENERAL_XVV-6320S_JH1706_20211203_v004-video-index0");
        this->declare_parameter("path_cam_screenshot", "/dev/v4l/by-id/usb-GENERAL_XVV-6320S_JH17cmvk06_20211203_v004-video-index0");

        this->declare_parameter("width_main", 640);
        this->declare_parameter("height_main", 480);

        this->declare_parameter("width_mini_arm", 640);
        this->declare_parameter("height_mini_arm", 480);

        this->declare_parameter("width_screenshot", 640);
        this->declare_parameter("height_screenshot", 480);

        this->declare_parameter("fps_main",30);
        this->declare_parameter("fps_mini_arm",30);

        this->declare_parameter("is_main_gray", true);
        this->declare_parameter("is_mini_arm_gray", true);

        //===============================================================================

        path_cam_main = this->get_parameter("path_cam_main").as_string();
        path_cam_mini_arm = this->get_parameter("path_cam_mini_arm").as_string();
        path_cam_screenshot = this->get_parameter("path_cam_screenshot").as_string();

        qos_state_main = this->get_parameter("qos_state_main").as_bool();
        qos_state_mini_arm = this->get_parameter("qos_state_mini_arm").as_bool();
        qos_state_screenshot = this->get_parameter("qos_state_screenshot").as_bool();

        topic_main = this->get_parameter("topic_main").as_string();
        topic_mini_arm = this->get_parameter("topic_mini_arm").as_string();
        topic_screenshot = this->get_parameter("topic_screenshot").as_string();

        pub_main = this->get_parameter("pub_main_compressed_image").as_bool();
        pub_mini_arm = this->get_parameter("pub_mini_arm_compressed_image").as_bool();
        pub_screenshot = this->get_parameter("pub_screenshot_compressed_image").as_bool();

        width_main = this->get_parameter("width_main").as_int();
        height_main = this->get_parameter("height_main").as_int();

        width_mini_arm = this->get_parameter("width_mini_arm").as_int();
        height_mini_arm = this->get_parameter("height_mini_arm").as_int();

        width_screenshot = this->get_parameter("width_screenshot").as_int();
        height_screenshot = this->get_parameter("height_screenshot").as_int();

        fps_main = this->get_parameter("fps_main").as_int();
        fps_mini_arm = this->get_parameter("fps_mini_arm").as_int();

        is_main_gray = this->get_parameter("is_main_gray").as_bool();
        is_mini_arm_gray = this->get_parameter("is_mini_arm_gray").as_bool();

        //DEBUG
        RCLCPP_INFO(this->get_logger(), "qos_state_main: %s", qos_state_main ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "qos_state_mini_arm: %s", qos_state_mini_arm ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "qos_state_screenshot: %s", qos_state_screenshot ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "topic_main: %s", topic_main.c_str());
        RCLCPP_INFO(this->get_logger(), "topic_mini_arm: %s", topic_mini_arm.c_str());
        RCLCPP_INFO(this->get_logger(), "topic_screenshot: %s", topic_screenshot.c_str());
        RCLCPP_INFO(this->get_logger(), "pub_main_compressed_image: %s", pub_main ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "pub_mini_arm_compressed_image: %s", pub_mini_arm ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "pub_screenshot_compressed_image: %s", pub_screenshot ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "path_cam_main: %s", path_cam_main.c_str());
        RCLCPP_INFO(this->get_logger(), "path_cam_mini_arm: %s", path_cam_mini_arm.c_str());
        RCLCPP_INFO(this->get_logger(), "path_cam_screenshot: %s", path_cam_screenshot.c_str());
        RCLCPP_INFO(this->get_logger(), "width_main: %d", width_main);
        RCLCPP_INFO(this->get_logger(), "height_main: %d", height_main);
        RCLCPP_INFO(this->get_logger(), "width_mini_arm: %d", width_mini_arm);
        RCLCPP_INFO(this->get_logger(), "height_mini_arm: %d", height_mini_arm);
        RCLCPP_INFO(this->get_logger(), "width_screenshot: %d", width_screenshot);
        RCLCPP_INFO(this->get_logger(), "height_screenshot: %d", height_screenshot);
        RCLCPP_INFO(this->get_logger(), "CAM MAIN IS: %s", is_main_gray?"GRAY":"BGR");
        RCLCPP_INFO(this->get_logger(), "CAM MINI ARM IS: %s", is_mini_arm_gray?"GRAY":"BGR");

        cap_main.open(path_cam_main ,cv::CAP_V4L2);
        cap_mini_arm.open(path_cam_mini_arm ,cv::CAP_V4L2);
        cap_screenshot.open(path_cam_screenshot, cv::CAP_V4L2);

        cap_main.set(cv::CAP_PROP_BUFFERSIZE, 1);
        cap_mini_arm.set(cv::CAP_PROP_BUFFERSIZE, 1);

        // CHNAGE REOLUTION==============================================================
        cap_main.set(cv::CAP_PROP_FRAME_WIDTH, width_main);
        cap_main.set(cv::CAP_PROP_FRAME_HEIGHT, height_main);

        cap_mini_arm.set(cv::CAP_PROP_FRAME_WIDTH, width_mini_arm);
        cap_mini_arm.set(cv::CAP_PROP_FRAME_HEIGHT, height_mini_arm);

        cap_screenshot.set(cv::CAP_PROP_FRAME_WIDTH, width_screenshot);
        cap_screenshot.set(cv::CAP_PROP_FRAME_HEIGHT, height_screenshot);

        // CHANGE FPS====================================================================
        cap_main.set(cv::CAP_PROP_FPS, 5);


        if(cap_screenshot.isOpened()) {
            cap_screenshot.set(cv::CAP_PROP_BUFFERSIZE, 1);  // Buffer minimal
        }

        // Timer untuk main camera
        timer_main = this->create_wall_timer(
            std::chrono::milliseconds(calc_fps(fps_main)) , std::bind(&arm_pov::timer_callback_main, this)
        );

        // Timer untuk mini arm camera
        timer_mini_arm = this->create_wall_timer(
            std::chrono::milliseconds(calc_fps(fps_mini_arm)) , std::bind(&arm_pov::timer_callback_mini_arm, this)
        );

        image_screenshot_sub = this->create_subscription<std_msgs::msg::Char>(
            "/imagescreehot",
            rclcpp::QoS(rclcpp::KeepLast(10))
                .reliability(rclcpp::ReliabilityPolicy::Reliable)
                .durability(rclcpp::DurabilityPolicy::Volatile),
            std::bind(&arm_pov::timer_callback_screenshot, this, std::placeholders::_1));

        param_callback_handler_ = this->add_on_set_parameters_callback(
            std::bind(&arm_pov::param_callback, this, std::placeholders::_1)
        );

        // QoS setting
        qos_output_publish(qos_main, qos_state_main);
        qos_output_publish(qos_mini_arm, qos_state_mini_arm);
        qos_output_publish(qos_screenshot, qos_state_screenshot);

        // Publisher setup
        publish_compressed(topic_main, pub_main, image_pub_main, image_pub_main_compressed, qos_main);
        publish_compressed(topic_mini_arm, pub_mini_arm, image_pub_mini_arm, image_pub_mini_armcompressed, qos_mini_arm);
        publish_compressed(topic_screenshot, pub_screenshot, image_pub_screenshot, image_pub_screenshotcompressed, qos_screenshot);

    }

private:
    rclcpp::TimerBase::SharedPtr timer_main;
    rclcpp::TimerBase::SharedPtr timer_mini_arm;

    // Wrapper supaya timer bisa panggil image_callback_x
    void timer_callback_main() {
        frame_main = image_callback_main(is_main_gray);
        if (frame_main.empty()) return;
        // cv::resize(frame_main,frame_main,cv::Size(width_main,height_main),cv::INTER_LINEAR);


        std_msgs::msg::Header header;
        header.stamp = this->now();

        if(is_main_gray){
            // COMPRESSED
            if (pub_main){
                auto comp_msg = cv_bridge::CvImage(header, "mono8", frame_main).toCompressedImageMsg(cv_bridge::JPG);
                image_pub_main_compressed->publish(*comp_msg);
            }

            // RAW
            else{
                auto msg = cv_bridge::CvImage(header, "mono8", frame_main).toImageMsg();
                image_pub_main->publish(*msg);
            }
        }else {
            // COMPRESSED
            if (pub_main){
                auto comp_msg = cv_bridge::CvImage(header, "bgr8", frame_main).toCompressedImageMsg(cv_bridge::JPG);
                image_pub_main_compressed->publish(*comp_msg);
            }

            // RAW
            else{
                auto msg = cv_bridge::CvImage(header, "bgr8", frame_main).toImageMsg();
                image_pub_main->publish(*msg);
            }
        }
    }

    void timer_callback_mini_arm() {
        frame_mini_arm = image_callback_mini_arm(is_mini_arm_gray);
        if (frame_mini_arm.empty()) return;

        std_msgs::msg::Header header;
        header.stamp = this->now();

        if(is_mini_arm_gray){
            // COMPRESSED
            if (pub_mini_arm) {
                auto comp_msg = cv_bridge::CvImage(header, "mono8", frame_mini_arm).toCompressedImageMsg(cv_bridge::JPG);
                image_pub_mini_armcompressed->publish(*comp_msg);
            }

            // RAW
            else {
                auto msg = cv_bridge::CvImage(header, "mono8", frame_mini_arm).toImageMsg();
                image_pub_mini_arm->publish(*msg);
            }
        }else{
            // COMPRESSED
            if (pub_mini_arm) {
                auto comp_msg = cv_bridge::CvImage(header, "bgr8", frame_mini_arm).toCompressedImageMsg(cv_bridge::JPG);
                image_pub_mini_armcompressed->publish(*comp_msg);
            }

            // RAW
            else {
                auto msg = cv_bridge::CvImage(header, "bgr8", frame_mini_arm).toImageMsg();
                image_pub_mini_arm->publish(*msg);
            }
        }
    }
    void timer_callback_screenshot(const std_msgs::msg::Char::ConstSharedPtr& msg) {
        RCLCPP_INFO(get_logger(),"Received screenshot command: %c", msg->data);
        if(msg->data == 's'){
            RCLCPP_INFO(get_logger(),"OK_SCREENSHOT");
            frame_screenshot = image_callback_screenshot();
            if (frame_screenshot.empty()) {
                RCLCPP_WARN(get_logger(), "Screenshot frame is empty!");
                return;
            }

            std_msgs::msg::Header header;
            header.stamp = this->now();

            // COMPRESSED
            if (pub_screenshot) {
                if (image_pub_screenshotcompressed) {
                    auto comp_msg = cv_bridge::CvImage(header, "mono8", frame_screenshot).toCompressedImageMsg(cv_bridge::JPG);
                    image_pub_screenshotcompressed->publish(*comp_msg);
                    RCLCPP_INFO(get_logger(), "Published COMPRESSED screenshot to: %s/compressed", topic_screenshot.c_str());
                } else {
                    RCLCPP_ERROR(get_logger(), "Compressed publisher is null!");
                }
            }

            // RAW
            else{
                if (image_pub_screenshot) {
                    auto msg_img = cv_bridge::CvImage(header, "mono8", frame_screenshot).toImageMsg();
                    image_pub_screenshot->publish(*msg_img);
                    RCLCPP_INFO(get_logger(), "Published RAW screenshot to: %s", topic_screenshot.c_str());
                } else {
                    RCLCPP_ERROR(get_logger(), "Raw publisher is null!");
                }
            }
        }
    }

    cv::Mat image_callback_main(const bool& is_gray);
    cv::Mat image_callback_mini_arm(const bool& is_gray);
    cv::Mat image_callback_screenshot();

    //FUNCTION===========================================================================

    void publish_compressed(std::string& name_topic, bool& state,
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr&,
        rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr&, 
        rclcpp::QoS& qos_);

    rclcpp::QoS qos_output_publish(rclcpp::QoS& Out, bool& state);

    void raw_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg, 
                      rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_pub_, 
                      rclcpp::Logger logger);

    void compressed_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& msg, 
                             rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_, 
                             rclcpp::Logger logger);

    rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> &params);
    //====================================================================================

    cv::VideoCapture cap_main;
    cv::VideoCapture cap_mini_arm;
    cv::VideoCapture cap_screenshot;

    std::string path_cam_main;
    std::string path_cam_mini_arm;
    std::string path_cam_screenshot;

    rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr image_screenshot_sub;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_main;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_screenshot;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_mini_arm;

    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_pub_main_compressed;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_pub_screenshotcompressed;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_pub_mini_armcompressed;

    rclcpp::QoS qos_main{10};
    rclcpp::QoS qos_mini_arm{10};
    rclcpp::QoS qos_screenshot{10};

    cv::Mat frame_main;
    cv::Mat frame_mini_arm;
    cv::Mat frame_screenshot;

    // bool output_besteffort_main = false;
    // bool output_besteffort_mini_arm = false;
    // bool output_besteffort_screenshot = false;
    
    bool qos_state_main = false;
    bool qos_state_mini_arm = false;
    bool qos_state_screenshot = false;

    std::string topic_main = "/image_main";
    std::string topic_mini_arm = "/image_mini_arm";
    std::string topic_screenshot = "/image_screenshot";

    bool pub_main = false;
    bool pub_mini_arm = false;
    bool pub_screenshot = false;

    int width_main = 640;
    int height_main = 480;

    int width_mini_arm = 640;
    int height_mini_arm = 480;

    int width_screenshot = 640;
    int height_screenshot = 480;

    int fps_main = 30;
    int fps_mini_arm = 30;

    //additional functions
    int calc_fps(int set_fps);
    
    bool is_main_gray = true;
    bool is_mini_arm_gray = true;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handler_;

};

//PARAM HANDLERERRRR

rcl_interfaces::msg::SetParametersResult arm_pov::param_callback(const std::vector<rclcpp::Parameter> &params){
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for(const auto &param : params){
        if(param.get_name() == "fps_main"){
            int new_fps = param.as_int();
            if(new_fps > 0){
                fps_main = new_fps;
                RCLCPP_INFO(this->get_logger(),"[SUCCES] PARAM frame_main: %d",fps_main);

                if(timer_main){
                    timer_main->cancel();
                }

                timer_main = this->create_wall_timer(std::chrono::milliseconds(calc_fps(fps_main)),std::bind(&arm_pov::timer_callback_main, this));
            }else RCLCPP_INFO(get_logger(),"INVALID FPS VALUE");
        }else if(param.get_name() == "fps_mini_arm"){
            int new_fps = param.as_int();
            if(new_fps > 0){
                fps_mini_arm = new_fps;
                RCLCPP_INFO(this->get_logger(),"[SUCCES] PARAM frame_main: %d",fps_mini_arm);

                if(timer_mini_arm){
                    timer_mini_arm->cancel();
                }

                timer_mini_arm = this->create_wall_timer(std::chrono::milliseconds(calc_fps(fps_mini_arm)),std::bind(&arm_pov::timer_callback_mini_arm, this));
            }else RCLCPP_INFO(get_logger(),"INVALID FPS VALUE");
        }else if(param.get_name() == "is_main_gray"){
            bool new_param = param.as_bool();
            if(new_param != is_main_gray){
                is_main_gray = new_param;
            }
        }else if(param.get_name() == "is_mini_arm_gray"){
            bool new_param = param.as_bool();
            if(new_param != is_mini_arm_gray){
                is_mini_arm_gray = new_param;
            }
        }
        // else if(param.get_name() == "width_main"){
        //     cap_main.set(cv::CAP_PROP_FRAME_WIDTH, width_main);
        // }
        // else if(param.get_name() == "height_main"){
        //     cap_main.set(cv::CAP_PROP_FRAME_WIDTH, height_main);
        // }//sek bingung
    }
    return result;
}


int arm_pov::calc_fps(int set_fps){
    return 1000/set_fps;
}

void arm_pov::raw_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg, 
                           rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_pub_, 
                           rclcpp::Logger logger) {
    raw_pub_->publish(*msg);
    static int msg_count = 0;
    if (++msg_count % 100 == 0) {
        RCLCPP_INFO(logger, "Relayed %d raw images", msg_count);
    }
}

void arm_pov::compressed_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& msg, 
                                  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_, 
                                  rclcpp::Logger logger) {
    compressed_pub_->publish(*msg);
    static int msg_count = 0;
    if (++msg_count % 100 == 0) {
      RCLCPP_INFO(logger, "Relayed %d compressed images", msg_count);
    }
}

rclcpp::QoS arm_pov::qos_output_publish(rclcpp::QoS& Out, bool& state){
    if (state) {
        Out.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    } else {
        Out.reliability(rclcpp::ReliabilityPolicy::Reliable);
    }
    return Out;
}

void arm_pov::publish_compressed(std::string& name_topic, bool& state, 
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& topic_not_compress, 
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr& topic_compress, 
    rclcpp::QoS& qos_){

    if(state){
      // Buat publisher untuk compressed image
      topic_compress = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        name_topic + "/compressed", qos_);
      RCLCPP_INFO(this->get_logger(), "Created COMPRESSED publisher for: %s/compressed", name_topic.c_str());
    }else{
      // Buat publisher untuk raw image  
      topic_not_compress = this->create_publisher<sensor_msgs::msg::Image>(
        name_topic, qos_);
      RCLCPP_INFO(this->get_logger(), "Created RAW publisher for: %s", name_topic.c_str());
    }
}

cv::Mat arm_pov::image_callback_main(const bool& is_gray){
    if(!cap_main.isOpened()){
        return cv::Mat();
    }
    cv::Mat frame;
    cap_main >> frame;
    // RCLCPP_INFO(this->get_logger(),"COL: %d, ROW: %d",frame.cols,frame.rows);
    if(frame.empty()){
        RCLCPP_WARN(this->get_logger(), "Empty frame captured! FOR MAIN");
        return cv::Mat();
    }

    if(is_gray){
        cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
    }

    return frame;
}

cv::Mat arm_pov::image_callback_mini_arm(const bool& is_gray){
    if(!cap_mini_arm.isOpened()){
        return cv::Mat();
    }
    cv::Mat frame;
    cap_mini_arm >> frame;

    // RCLCPP_INFO(this->get_logger(),"COL: %d, ROW: %d",frame.cols,frame.rows);

    if(frame.empty()){
        RCLCPP_WARN(this->get_logger(), "Empty frame captured! FOR MINI ARM");
        return cv::Mat();
    }

    if(is_gray){
        cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
    }
    
    return frame;
}

// cv::Mat arm_pov::image_callback_screenshot(){
//     if(!cap_screenshot.isOpened()){
//         return cv::Mat();
//     }
//     cv::Mat frame;
//     cap_screenshot >> frame;
//     if(frame.empty()){
//         RCLCPP_WARN(this->get_logger(), "Empty frame captured! FOR SCREENSHOT");
//         return cv::Mat();
//     }
//     cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
//     return frame;
// }

cv::Mat arm_pov::image_callback_screenshot(){
    if(!cap_screenshot.isOpened()){
        return cv::Mat();
    }
    
    cv::Mat frame;
    
    // Flush 2-3 frame untuk memastikan fresh
    for(int i = 0; i < 3; i++) {
        cap_screenshot >> frame;
    }
    
    if(frame.empty()){
        RCLCPP_WARN(this->get_logger(), "Empty frame captured! FOR SCREENSHOT");
        return cv::Mat();
    }
    
    cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
    return frame;
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<arm_pov>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
