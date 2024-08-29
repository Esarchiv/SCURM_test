#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include "rclcpp/rclcpp.hpp" 
#include <cv_bridge/cv_bridge.h> 
#include <vector> 

class ImageProcessor : public rclcpp::Node
{
public:
    ImageProcessor() : Node("image_processor"), frame_count_(0), fps_(0.0)
    {
        // 设置QoS策略
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));

        
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", qos,
            std::bind(&ImageProcessor::image_callback, this, std::placeholders::_1));

        
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/image_fps", qos);

        // 创建定时器
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&ImageProcessor::timer_callback, this));
    }

private:
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
    {
        // 将ROS图像消息转换为OpenCV图像
        cv::Mat img;
        try
        {
            img = cv_bridge::toCvShare(img_msg, "rgb8")->image;
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // 帧计数
        frame_count_++;

        // 在图像上显示帧率
        std::string fps_text = "FPS: " + std::to_string(fps_);
        cv::putText(img, fps_text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, CV_RGB(255, 0, 0), 2);


        auto img_msg_out = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", img).toImageMsg();
        publisher_->publish(*img_msg_out);
    }

    void timer_callback()
    {

        fps_ = frame_count_;
        frame_count_ = 0; 
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_; 
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_; 
    rclcpp::TimerBase::SharedPtr timer_; 
    int frame_count_; 
    double fps_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<ImageProcessor>()); 
    rclcpp::shutdown(); 
    return 0;
}