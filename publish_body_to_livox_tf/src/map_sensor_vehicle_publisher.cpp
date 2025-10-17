#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// map->sensor

class MapSensorVehiclePublisher : public rclcpp::Node
{
public:
    MapSensorVehiclePublisher()
    : Node("map_sensor_vehicle_publisher"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_),
      tf_broadcaster_(this),
      initialized_(false)
    {
        using namespace std::chrono_literals;
        timer_ = this->create_wall_timer(50ms, std::bind(&MapSensorVehiclePublisher::timerCallback, this));
        RCLCPP_INFO(this->get_logger(), "✅ map_sensor_vehicle_publisher started.");
    }

private:
    void timerCallback()
    {
        geometry_msgs::msg::TransformStamped base_to_livox;
        try
        {
            base_to_livox = tf_buffer_.lookupTransform("base_link", "livox_frame", tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                                 "TF lookup failed: %s", ex.what());
            return;
        }

        // 初始化第一帧 map->sensor
        if (!initialized_)
        {
            first_livox_pose_ = base_to_livox;
            initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "📍 First frame set as map origin (livox_frame).");
        }

        // 1️⃣ map -> sensor (固定)
        geometry_msgs::msg::TransformStamped map_to_sensor;
        map_to_sensor.header.stamp = this->now();
        map_to_sensor.header.frame_id = "map";
        map_to_sensor.child_frame_id = "sensor";
        map_to_sensor.transform.translation.x = 0.0;
        map_to_sensor.transform.translation.y = 0.0;
        map_to_sensor.transform.translation.z = 0.0;
        map_to_sensor.transform.rotation.x = 0.0;
        map_to_sensor.transform.rotation.y = 0.0;
        map_to_sensor.transform.rotation.z = 0.0;
        map_to_sensor.transform.rotation.w = 1.0;
        tf_broadcaster_.sendTransform(map_to_sensor);

        // 2️⃣ sensor -> vehicle (由 base_link->livox_frame 取逆)
        tf2::Transform tf_base_to_livox;
        tf2::fromMsg(base_to_livox.transform, tf_base_to_livox);
        tf2::Transform tf_livox_to_base = tf_base_to_livox.inverse();

        geometry_msgs::msg::TransformStamped sensor_to_vehicle;
        sensor_to_vehicle.header.stamp = this->now();
        sensor_to_vehicle.header.frame_id = "sensor";
        sensor_to_vehicle.child_frame_id = "vehicle";
        sensor_to_vehicle.transform = tf2::toMsg(tf_livox_to_base);
        tf_broadcaster_.sendTransform(sensor_to_vehicle);
    }

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::TransformStamped first_livox_pose_;
    bool initialized_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapSensorVehiclePublisher>());
    rclcpp::shutdown();
    return 0;
}
