// 文件：src/publish_body_to_dummy_tf.cpp
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class BodyToDummyTfPublisher : public rclcpp::Node
{
public:
  BodyToDummyTfPublisher()
  : Node("body_to_dummy_tf_publisher")
  {
    // 创建 StaticTransformBroadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // 设置 body → dummy 的恒等变换
    transform_.header.frame_id = "body";
    transform_.child_frame_id = "dummy";
    transform_.transform.translation.x = -0.12;
    transform_.transform.translation.y = 0.0;
    transform_.transform.translation.z = -0.175;
    transform_.transform.rotation.x = 0.0;
    transform_.transform.rotation.y = 0.0;
    transform_.transform.rotation.z = 0.0;
    transform_.transform.rotation.w = 1.0;

    // 每 1 秒发布一次（可改为 0.1s ~ 5s）
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),  // 每 100ms 发布一次（即 10 Hz）
        [this]() {
            transform_.header.stamp = this->get_clock()->now();
            tf_broadcaster_->sendTransform(transform_);
        }
    );

    RCLCPP_INFO(this->get_logger(), "Publishing: body → dummy continuously at 1 Hz");
  }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::TransformStamped transform_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BodyToDummyTfPublisher>());
  rclcpp::shutdown();
  return 0;
}