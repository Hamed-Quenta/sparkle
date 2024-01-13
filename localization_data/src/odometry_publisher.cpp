#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "cmath"

using namespace std::chrono_literals;

class OdometryPublisher : public rclcpp::Node
{
public:
  OdometryPublisher()
    : Node("odometry_publisher")
  {
    odom_data_pub_quat_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_data_quat", 10);
    odom_data_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_data_euler", 10);
    
    auto qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    
    // Initialize the transform broadcaster
    // tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    timer_ = this->create_wall_timer(
      10ms, std::bind(&OdometryPublisher::timer_callback, this));

    // Initialize subscribers
    left_ticks_subscription_ = this->create_subscription<std_msgs::msg::Int16>("left_ticks", qos_profile, std::bind(&OdometryPublisher::left_ticks_callback, this, std::placeholders::_1));
    right_ticks_subscription_ = this->create_subscription<std_msgs::msg::Int16>("right_ticks", qos_profile, std::bind(&OdometryPublisher::right_ticks_callback, this, std::placeholders::_1));
    initial_2d_suscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10, std::bind(&OdometryPublisher::initial_pose_callback, this, std::placeholders::_1));
  }

private:
    // Initial pose
    const double initialX = 0.0;
    const double initialY = 0.0;
    const double initialTheta = 0.00000000001;
    const double PI = 3.141592;

    // Odometry messages
    nav_msgs::msg::Odometry odom_old;
    nav_msgs::msg::Odometry odom_new;
    
    // Robot physical constants
    const double TICKS_PER_REVOLUTION = 8400;
    const double WHEEL_RADIUS = 0.05;
    const double WHEEL_BASE = 0.21;
    const double TICKS_PER_METER = 82500;

    // Distance both wheels have traveled
    double distance_left = 0;
    double distance_right = 0;

    // Flag to see if initial pose has been received
    bool initialPoseReceived = false;

    void initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        // Set the initial pose
        odom_old.pose.pose.position.x = msg->pose.pose.position.x;
        odom_old.pose.pose.position.y = msg->pose.pose.position.y;
        odom_old.pose.pose.orientation.z = msg->pose.pose.orientation.z;

        // Set the initial pose flag
        initialPoseReceived = true;
    }

    void left_ticks_callback(const std_msgs::msg::Int16::SharedPtr msg)
    {
        static int last_count_left = 0;
        if (msg->data != 0 && last_count_left != 0)
        {
            int left_ticks = msg->data - last_count_left;

            if (left_ticks > 10000) {
              left_ticks -= 65535;
            }
            else if (left_ticks < -10000) {
              left_ticks = 65535 - left_ticks;
            }
            else{}

            // Convert the number of ticks to meters
            distance_left = left_ticks / TICKS_PER_METER;
        }
        last_count_left = msg->data;
    }

    void right_ticks_callback(const std_msgs::msg::Int16::SharedPtr msg)
    {
        static int last_count_right = 0;
        if (msg->data != 0 && last_count_right != 0)
        {
            int right_ticks = msg->data - last_count_right;

            if (right_ticks > 10000) {
              right_ticks -= 65535;
            distance_right = (0 - (65535 - distance_right)) / TICKS_PER_METER;
            }
            else if (right_ticks < -10000) {
              right_ticks = 65535 - right_ticks;
            }
            else{}

            // Convert the number of ticks to meters
            distance_right = right_ticks / TICKS_PER_METER;
        }
        last_count_right = msg->data;
    }

    void publish_quat()
    {
        geometry_msgs::msg::TransformStamped t;
        tf2::Quaternion q;
        q.setRPY(0, 0, odom_new.pose.pose.orientation.z);

        auto odom_quat = nav_msgs::msg::Odometry();
        odom_quat.header.stamp = odom_new.header.stamp;
        odom_quat.header.frame_id = "odom";
        odom_quat.child_frame_id = "base_link";
        odom_quat.pose.pose.position.x = odom_new.pose.pose.position.x;
        odom_quat.pose.pose.position.y = odom_new.pose.pose.position.y;
        odom_quat.pose.pose.position.z = odom_new.pose.pose.position.z;
        odom_quat.pose.pose.orientation.x = q.x();
        odom_quat.pose.pose.orientation.y = q.y();
        odom_quat.pose.pose.orientation.z = q.z();
        odom_quat.pose.pose.orientation.w = q.w();
        odom_quat.twist.twist.linear.x = odom_new.twist.twist.linear.x;
        odom_quat.twist.twist.linear.y = odom_new.twist.twist.linear.y;
        odom_quat.twist.twist.linear.z = odom_new.twist.twist.linear.z;
        odom_quat.twist.twist.angular.x = odom_new.twist.twist.angular.x;
        odom_quat.twist.twist.angular.y = odom_new.twist.twist.angular.y;
        odom_quat.twist.twist.angular.z = odom_new.twist.twist.angular.z;

        for(int i = 0; i < 36; i++)
        {
            if(i == 0 || i == 7 || i == 14){
                odom_quat.pose.covariance[i] = 0.1;
            }
            else if(i == 21 || i == 28 || i == 35){
                odom_quat.pose.covariance[i] += 0.1;
            }
            else{
                odom_quat.pose.covariance[i] = 0;
            }
        }

        // t.header.stamp = odom_new.header.stamp;
        // t.header.frame_id = "odom";
        // t.child_frame_id = "base_link";
        // t.transform.translation.x = odom_new.pose.pose.position.x;
        // t.transform.translation.y = odom_new.pose.pose.position.y;
        // t.transform.translation.z = odom_new.pose.pose.position.z;
        // t.transform.rotation.x = q.x();
        // t.transform.rotation.y = q.y();
        // t.transform.rotation.z = q.z();
        // t.transform.rotation.w = q.w();

        odom_data_pub_quat_->publish(odom_quat);
        // tf_broadcaster_->sendTransform(t);
    }

    // Update odometry information
    void update_odometry()
    {
        double cycle_distance = (distance_left + distance_right) / 2;
        double cycle_angle = std::asin((distance_right - distance_left) / WHEEL_BASE);
        double avg_angle = odom_old.pose.pose.orientation.z + cycle_angle / 2; // TODO: Revisar

        if (avg_angle > PI)
            avg_angle -= 2 * PI;
        else if (avg_angle < -PI)
            avg_angle += 2 * PI;
        else{}

        odom_new.pose.pose.position.x = odom_old.pose.pose.position.x + cycle_distance * std::cos(avg_angle);
        odom_new.pose.pose.position.y = odom_old.pose.pose.position.y + cycle_distance * std::sin(avg_angle);
        odom_new.pose.pose.orientation.z = odom_old.pose.pose.orientation.z + cycle_angle;

        // Prevent lockup from a single bad cycle
        if (std::isnan(odom_new.pose.pose.position.x) || std::isnan(odom_new.pose.pose.position.y) || std::isnan(odom_new.pose.pose.position.z)) // TODO: revisar si es orientacion o posicion z
        {
            odom_new.pose.pose.position.x = odom_old.pose.pose.position.x;
            odom_new.pose.pose.position.y = odom_old.pose.pose.position.y;
            odom_new.pose.pose.orientation.z = odom_old.pose.pose.orientation.z;
        }

        // Make sure theta stays in the range [-pi, pi]
        if (odom_new.pose.pose.orientation.z > PI)
            odom_new.pose.pose.orientation.z -= 2 * PI;
        else if (odom_new.pose.pose.orientation.z < -PI)
            odom_new.pose.pose.orientation.z += 2 * PI;

        // Compute the velocitiesmap
        odom_new.header.stamp = rclcpp::Clock().now();
        odom_new.twist.twist.linear.x = cycle_distance / (odom_new.header.stamp.sec - odom_old.header.stamp.sec);
        odom_new.twist.twist.angular.z = cycle_angle / (odom_new.header.stamp.sec - odom_old.header.stamp.sec);

        // Save the pose data for the next cycle
        odom_old.pose.pose.position.x = odom_new.pose.pose.position.x;
        odom_old.pose.pose.position.y = odom_new.pose.pose.position.y;
        odom_old.pose.pose.orientation.z = odom_new.pose.pose.orientation.z;
        odom_old.header.stamp = odom_new.header.stamp;

        // Publish the odometry message
        odom_data_pub_->publish(odom_new);
    }

    void timer_callback()
    {
        if (initialPoseReceived)
        {
            update_odometry();
            publish_quat();
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_data_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_data_pub_quat_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr left_ticks_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr right_ticks_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_2d_suscription_;
    // std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdometryPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}