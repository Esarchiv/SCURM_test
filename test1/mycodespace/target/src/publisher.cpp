#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.h> 

#include <geometry_msgs/msg/point_stamped.hpp>
#include <deque>

class PointSubscriber : public rclcpp::Node
{
public:
    PointSubscriber() : Node("point_subscriber")
    {
        point_subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/target_point", 10,
            std::bind(&PointSubscriber::point_callback, this, std::placeholders::_1));

     
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PointSubscriber::timer_callback, this));
    }

private:
    void point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        latest_point_ = *msg;
    }

    void timer_callback()
    {
       
        if (latest_point_)
        {
            point_queue_.push_back(*latest_point_);
            latest_point_.reset(); 
        }

        
        if (point_queue_.size() > 10)
        {
            point_queue_.pop_front();
        }

        // 打印队列中的点
        RCLCPP_INFO(this->get_logger(), "Queue size: %zu", point_queue_.size());
        for (const auto &point : point_queue_)
        {
            RCLCPP_INFO(this->get_logger(), "Point: [%.2f, %.2f, %.2f]", point.point.x, point.point.y, point.point.z);
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_subscription_; 
    rclcpp::TimerBase::SharedPtr timer_; 
    std::deque<geometry_msgs::msg::PointStamped> point_queue_; 
    std::optional<geometry_msgs::msg::PointStamped> latest_point_; 
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<PointSubscriber>()); 
    rclcpp::shutdown(); 
    return 0;
}