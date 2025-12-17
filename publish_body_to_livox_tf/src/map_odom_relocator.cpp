#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

class MapOdomBasePublisher : public rclcpp::Node
{
public:
    MapOdomBasePublisher()
        : Node("map_odom_base_publisher"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // 订阅重定位初始位姿
        initialpose_sub_ = this->create_subscription<
            geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10,
            std::bind(&MapOdomBasePublisher::initialPoseCallback,
                      this, std::placeholders::_1));

        // 定时发布 TF
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&MapOdomBasePublisher::timerCallback, this));
    }

private:
    // ------------------- map → odom -------------------
    void initialPoseCallback(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(),
                    "Received initialpose, updating map->odom transform");

        // 不使用消息时间戳，用当前节点时间
        map_to_odom_.header.frame_id = "map";
        map_to_odom_.child_frame_id  = "odom";

        map_to_odom_.transform.translation.x = msg->pose.pose.position.x;
        map_to_odom_.transform.translation.y = msg->pose.pose.position.y;
        map_to_odom_.transform.translation.z = msg->pose.pose.position.z;

        map_to_odom_.transform.rotation = msg->pose.pose.orientation;

        has_initialpose_ = true;
    }

    // ------------------- timer: 发布全部 TF -------------------
    void timerCallback()
    {
        if (!has_initialpose_) return;

        auto now = this->now();

        // 1. 发布 map → odom （时间戳必须用 now()）
        map_to_odom_.header.stamp = now;
        tf_broadcaster_->sendTransform(map_to_odom_);

        // 2. 查询 camera_init → body（Fast-LIO 固定初始 TF）
        geometry_msgs::msg::TransformStamped cam_to_body;
        try {
            cam_to_body = tf_buffer_.lookupTransform(
                "camera_init", "body", tf2::TimePointZero);
        } catch (...) {
            RCLCPP_WARN(this->get_logger(),
                        "Cannot find TF camera_init -> body");
            return;
        }

        // 3. 查询 base_link → livox_frame（URDF静态TF）
        geometry_msgs::msg::TransformStamped base_to_livox;
        try {
            base_to_livox = tf_buffer_.lookupTransform(
                "base_link", "livox_frame", tf2::TimePointZero);
        } catch (...) {
            RCLCPP_WARN(this->get_logger(),
                        "Cannot find TF base_link -> livox_frame");
            return;
        }

        // ------------------- 计算 odom → base_link -------------------
        tf2::Transform T_cam_body, T_base_livox;
        tf2::fromMsg(cam_to_body.transform, T_cam_body);
        tf2::fromMsg(base_to_livox.transform, T_base_livox);

        tf2::Transform T_livox_base = T_base_livox.inverse();
        tf2::Transform T_odom_base  = T_cam_body * T_livox_base;

        geometry_msgs::msg::TransformStamped odom_to_base;
        odom_to_base.header.stamp = now;
        odom_to_base.header.frame_id = "odom";
        odom_to_base.child_frame_id = "base_link";
        odom_to_base.transform = tf2::toMsg(T_odom_base);

        tf_broadcaster_->sendTransform(odom_to_base);
    }

    // TF
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // map→odom
    bool has_initialpose_ = false;
    geometry_msgs::msg::TransformStamped map_to_odom_;

    // timer
    rclcpp::TimerBase::SharedPtr timer_;

    // subscriber
    rclcpp::Subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        initialpose_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapOdomBasePublisher>());
    rclcpp::shutdown();
    return 0;
}
