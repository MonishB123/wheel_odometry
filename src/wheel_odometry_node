#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

class WheelOdometry : public rclcpp::Node {
public:
    WheelOdometry() : Node("wheel_odometry"),
                      x_(0.0), y_(0.0), theta_(0.0),
                      last_left_ticks_(0), last_right_ticks_(0)
    {
        wheel_radius_ = declare_parameter("wheel_radius", 0.05);
        wheel_base_ = declare_parameter("wheel_base", 0.30);
        ticks_per_rev_ = declare_parameter("ticks_per_revolution", 2048);
        odom_frame_ = declare_parameter("odom_frame", "odom");
        base_frame_ = declare_parameter("base_frame", "base_link");

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

        encoder_sub_ = create_subscription<std_msgs::msg::Int64MultiArray>(
            "wheel_ticks", 10,
            std::bind(&WheelOdometry::encoderCallback, this, std::placeholders::_1)
        );

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        last_time_ = now();
        RCLCPP_INFO(get_logger(), "Wheel odometry node started");
    }

private:
    void encoderCallback(const std_msgs::msg::Int64MultiArray::SharedPtr msg) {
        if (msg->data.size() < 2) return;

        auto current_time = now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        int64_t left_ticks = msg->data[0];
        int64_t right_ticks = msg->data[1];

        int64_t delta_left = left_ticks - last_left_ticks_;
        int64_t delta_right = right_ticks - last_right_ticks_;

        last_left_ticks_ = left_ticks;
        last_right_ticks_ = right_ticks;

        double left_dist =
            2.0 * M_PI * wheel_radius_ * (double(delta_left) / ticks_per_rev_);
        double right_dist =
            2.0 * M_PI * wheel_radius_ * (double(delta_right) / ticks_per_rev_);

        double distance = (left_dist + right_dist) / 2.0;
        double delta_theta = (right_dist - left_dist) / wheel_base_;

        x_ += distance * cos(theta_ + delta_theta / 2.0);
        y_ += distance * sin(theta_ + delta_theta / 2.0);
        theta_ += delta_theta;

        publishOdometry(current_time, distance / dt, delta_theta / dt);
    }

    void publishOdometry(const rclcpp::Time &time, double linear_vel, double angular_vel) {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = time;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = base_frame_;

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        odom.twist.twist.linear.x = linear_vel;
        odom.twist.twist.angular.z = angular_vel;

        odom_pub_->publish(odom);

        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = time;
        tf.header.frame_id = odom_frame_;
        tf.child_frame_id = base_frame_;
        tf.transform.translation.x = x_;
        tf.transform.translation.y = y_;
        tf.transform.rotation = odom.pose.pose.orientation;

        tf_broadcaster_->sendTransform(tf);
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr encoder_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    double wheel_radius_, wheel_base_;
    int ticks_per_rev_;
    std::string odom_frame_, base_frame_;

    double x_, y_, theta_;
    int64_t last_left_ticks_, last_right_ticks_;
    rclcpp::Time last_time_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelOdometry>());
    rclcpp::shutdown();
    return 0;
}
