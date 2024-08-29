#include <rclcpp/rclcpp.hpp> 
#include <geometry_msgs/msg/point_stamped.hpp> 
#include <visualization_msgs/msg/marker.hpp> 
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h> 
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <deque> 

class transformer : public rclcpp::Node
{
public:
    transformer() : Node("point_transformer"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        
        point_subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/target_point", 10,
            std::bind(&transformer::point_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&transformer::timer_callback, this));

        // 创建Marker发布器
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
    }

private:

    void point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
      
        if (msg->header.frame_id.empty())
        {
            //RCLCPP_WARN(this->get_logger(), "Received point with empty frame_id, setting default frame_id to 'camera_link'");
            msg->header.frame_id = "camera_link"; // 设置默认帧ID
        }

        
        geometry_msgs::msg::PointStamped transformed_point;
        transformed_point.header = msg->header;
        transformed_point.point.x = msg->point.x;
        transformed_point.point.y = -msg->point.y;
        transformed_point.point.z = -msg->point.z;

        
        try
        {
            geometry_msgs::msg::PointStamped base_link_point;
            
            tf_buffer_.transform(transformed_point, base_link_point, "base_link", tf2::durationFromSec(0.0));

            
            point_queue_.push_back(base_link_point);
        }
        catch (tf2::TransformException &ex)
        {
            //RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
        }
    }

    void timer_callback()
    {
        if (point_queue_.size() < 2)
        {
            return;
        }

        auto start_point = point_queue_.front();
        point_queue_.pop_front();
        auto end_point = point_queue_.front();

        // 创建箭头类型的Marker
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = this->now();
        marker.ns = "arrows";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // 设置箭头的起点和终点
        geometry_msgs::msg::Point p_start, p_end;
        p_start.x = start_point.point.x;
        p_start.y = start_point.point.y;
        p_start.z = start_point.point.z;
        p_end.x = end_point.point.x;
        p_end.y = end_point.point.y;
        p_end.z = end_point.point.z;
        marker.points.push_back(p_start);
        marker.points.push_back(p_end);

        // 设置箭头的尺寸和颜色
        marker.scale.x = 0.05; // 箭头的杆的直径
        marker.scale.y = 0.1;  // 箭头的头的直径
        marker.scale.z = 0.1;  // 箭头的头的长度
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        marker_publisher_->publish(marker);
    }

 
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_subscription_;
   
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    
    tf2_ros::Buffer tf_buffer_;
   
    tf2_ros::TransformListener tf_listener_;
   
    std::deque<geometry_msgs::msg::PointStamped> point_queue_;
};

int main(int argc, char **argv)
{
    
    rclcpp::init(argc, argv);
    
    rclcpp::spin(std::make_shared<transformer>());
    
    rclcpp::shutdown();
    return 0;
}