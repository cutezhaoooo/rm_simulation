#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class MapOdomBaseBroadcaster : public rclcpp::Node
{
public:
    MapOdomBaseBroadcaster()
    : Node("map_odom_base_broadcaster")
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&MapOdomBaseBroadcaster::timerCallback, this));

        declare_parameter<bool>("use_identity_map_camera_init", true);
        declare_parameter<bool>("use_identity_body_base_link", true);

        use_identity_map_camera_init_ = get_parameter("use_identity_map_camera_init").as_bool();
        use_identity_body_base_link_ = get_parameter("use_identity_body_base_link").as_bool();

        RCLCPP_INFO(this->get_logger(), "✅ map_odom_base_broadcaster started.");
    }

private:
    void timerCallback()
    {
        geometry_msgs::msg::TransformStamped t_map_camera_init;
        geometry_msgs::msg::TransformStamped t_camera_init_body;
        geometry_msgs::msg::TransformStamped t_body_base_link;

        try
        {
            // map->camera_init
            if (!use_identity_map_camera_init_)
                t_map_camera_init = tf_buffer_->lookupTransform("map", "camera_init", tf2::TimePointZero);
            else
                t_map_camera_init = identityTransform("map", "camera_init");

            // camera_init->body（FastLIO输出）
            t_camera_init_body = tf_buffer_->lookupTransform("camera_init", "body", tf2::TimePointZero);

            // body->base_link（如果不存在，可以是单位变换）
            if (!use_identity_body_base_link_)
                t_body_base_link = tf_buffer_->lookupTransform("body", "base_link", tf2::TimePointZero);
            else
                t_body_base_link = identityTransform("body", "base_link");
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "TF lookup failed: %s", ex.what());
            return;
        }

        // === ⚙️ 分离两条 TF 树 ===
        //
        // FastLIO 输出的: camera_init -> body
        // 我们自己生成独立坐标: map -> odom -> base_link
        // 逻辑：odom 始终跟随 body，但帧名不同，保证 TF 树不连通

        // map->odom = map->camera_init * camera_init->body
        tf2::Transform T_map_camera_init, T_camera_init_body, T_map_odom;
        tf2::fromMsg(t_map_camera_init.transform, T_map_camera_init);
        tf2::fromMsg(t_camera_init_body.transform, T_camera_init_body);
        T_map_odom = T_map_camera_init * T_camera_init_body;

        geometry_msgs::msg::TransformStamped t_map_odom;
        t_map_odom.header.stamp = this->get_clock()->now();
        t_map_odom.header.frame_id = "map";
        t_map_odom.child_frame_id = "odom";
        t_map_odom.transform = tf2::toMsg(T_map_odom);
        tf_broadcaster_->sendTransform(t_map_odom);

        // odom->base_link：直接使用 body->base_link 的相对关系
        tf2::Transform T_body_base_link;
        tf2::fromMsg(t_body_base_link.transform, T_body_base_link);

        geometry_msgs::msg::TransformStamped t_odom_base;
        t_odom_base.header.stamp = this->get_clock()->now();
        t_odom_base.header.frame_id = "odom";
        t_odom_base.child_frame_id = "base_link";
        t_odom_base.transform = tf2::toMsg(T_body_base_link);
        tf_broadcaster_->sendTransform(t_odom_base);
    }

    geometry_msgs::msg::TransformStamped identityTransform(const std::string &parent, const std::string &child)
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.frame_id = parent;
        t.child_frame_id = child;
        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;
        t.transform.rotation.w = 1.0;
        return t;
    }

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool use_identity_map_camera_init_;
    bool use_identity_body_base_link_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapOdomBaseBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
