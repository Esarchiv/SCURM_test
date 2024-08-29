#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <auto_aim_interfaces/msg/target.hpp>

class EnemyPositionPublisher : public rclcpp::Node
{
public:
  EnemyPositionPublisher()
  : Node("enemy_position_publisher")
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    target_publisher_ = this->create_publisher<auto_aim_interfaces::msg::Target>("/tracker/target", 15);

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&EnemyPositionPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    try {
      auto now = this->get_clock()->now();

      geometry_msgs::msg::TransformStamped transformStamped;

      transformStamped = tf_buffer_->lookupTransform("yaw_link", "enemy", tf2::TimePointZero);

      auto_aim_interfaces::msg::Target target_msg;

      target_msg.position.x = transformStamped.transform.translation.x;
      target_msg.position.y = transformStamped.transform.translation.y;
      target_msg.position.z = transformStamped.transform.translation.z;

      target_msg.tracking = true;

      target_msg.header.stamp = now;

      target_msg.id = "114514";  

      target_publisher_->publish(target_msg);

    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
    }
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;  
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;  
  rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr target_publisher_;  
  rclcpp::TimerBase::SharedPtr timer_;  
};


int main(int argc, char * argv[])
{
  
  rclcpp::init(argc, argv);

  
  rclcpp::spin(std::make_shared<EnemyPositionPublisher>());

 
  rclcpp::shutdown();
  return 0;
}
