#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

class MapOdomBasePublisher : public rclcpp::Node
{
public:
    MapOdomBasePublisher() : Node("map_odom_base_publisher"),
                             tf_buffer_(this->get_clock()),
                             tf_listener_(tf_buffer_)
    {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        this->set_parameter(rclcpp::Parameter("use_sim_time", false));

        // 发布 map->odom 静态变换（恒等）
        geometry_msgs::msg::TransformStamped static_tf;
        static_tf.header.stamp = this->now();
        static_tf.header.frame_id = "map";
        static_tf.child_frame_id = "odom";
        static_tf.transform.translation.x = 0.0;
        static_tf.transform.translation.y = 0.0;
        static_tf.transform.translation.z = 0.0;
        static_tf.transform.rotation.x = 0.0;
        static_tf.transform.rotation.y = 0.0;
        static_tf.transform.rotation.z = 0.0;
        static_tf.transform.rotation.w = 1.0;
        static_broadcaster_->sendTransform(static_tf);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&MapOdomBasePublisher::timerCallback, this));
    }

private:
    void timerCallback()
    {
        geometry_msgs::msg::TransformStamped camera_to_body;
        geometry_msgs::msg::TransformStamped base_to_livox;

        try
        {
            camera_to_body = tf_buffer_.lookupTransform("camera_init", "body", tf2::TimePointZero);
            base_to_livox = tf_buffer_.lookupTransform("base_link", "livox_frame", tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
            return;
        }

        // 转换为 tf2 类型便于计算
        tf2::Transform T_camera_body, T_base_livox;
        tf2::fromMsg(camera_to_body.transform, T_camera_body);
        tf2::fromMsg(base_to_livox.transform, T_base_livox);

        // 计算 T_odom_base = T_camera_body * T_livox_base
        tf2::Transform T_livox_base = T_base_livox.inverse();
        tf2::Transform T_odom_base = T_camera_body * T_livox_base;

        // 发布 odom->base_link
        geometry_msgs::msg::TransformStamped odom_to_base;
        odom_to_base.header.stamp = this->now();
        odom_to_base.header.frame_id = "odom";
        odom_to_base.child_frame_id = "base_link";
        odom_to_base.transform = tf2::toMsg(T_odom_base);

        tf_broadcaster_->sendTransform(odom_to_base);
    }

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapOdomBasePublisher>());
    
    rclcpp::shutdown();
    return 0;
}
