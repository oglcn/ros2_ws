/**
 * ROS2 wrapper for ORB-SLAM3 monocular mode.
 *
 * Subscribes to camera images, runs ORB-SLAM3 tracking, and publishes
 * visual odometry (nav_msgs/Odometry), TF (odom -> base_link), and
 * the sparse map point cloud (sensor_msgs/PointCloud2).
 */

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <opencv2/core.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "System.h"
#include "MapPoint.h"

static constexpr size_t MAX_POINTCLOUD_POINTS = 3000;

class OrbSlam3Node : public rclcpp::Node {
public:
    OrbSlam3Node() : Node("orb_slam3_node") {
        declare_parameter("vocabulary_file", "");
        declare_parameter("settings_file", "");
        declare_parameter("odom_frame", "odom");
        declare_parameter("base_frame", "base_link");
        declare_parameter("camera_frame", "camera_link");

        vocab_file_ = get_parameter("vocabulary_file").as_string();
        settings_file_ = get_parameter("settings_file").as_string();
        odom_frame_ = get_parameter("odom_frame").as_string();
        base_frame_ = get_parameter("base_frame").as_string();
        camera_frame_ = get_parameter("camera_frame").as_string();

        if (vocab_file_.empty() || settings_file_.empty()) {
            RCLCPP_FATAL(get_logger(),
                "vocabulary_file and settings_file parameters are required");
            throw std::runtime_error("Missing required parameters");
        }

        RCLCPP_INFO(get_logger(), "Initializing ORB-SLAM3...");
        RCLCPP_INFO(get_logger(), "  Vocabulary: %s", vocab_file_.c_str());
        RCLCPP_INFO(get_logger(), "  Settings: %s", settings_file_.c_str());

        slam_system_ = std::make_unique<ORB_SLAM3::System>(
            vocab_file_, settings_file_,
            ORB_SLAM3::System::MONOCULAR, false /* no viewer */
        );

        RCLCPP_INFO(get_logger(), "ORB-SLAM3 initialized successfully");

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
            "visual_odom", 10);

        pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "vslam/map_points", 10);

        auto qos = rclcpp::QoS(1).best_effort();
        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "camera/image_raw", qos,
            std::bind(&OrbSlam3Node::image_callback, this, std::placeholders::_1));

        pointcloud_timer_ = create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&OrbSlam3Node::publish_map_points, this));

        RCLCPP_INFO(get_logger(), "ORB-SLAM3 node ready, waiting for images...");
    }

    ~OrbSlam3Node() override {
        if (slam_system_) {
            slam_system_->Shutdown();
        }
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        Sophus::SE3f pose = slam_system_->TrackMonocular(cv_ptr->image, timestamp);

        int state = slam_system_->GetTrackingState();
        if (state != 2) { // 2 = OK
            if (state == 3) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                    "ORB-SLAM3: tracking lost, attempting relocalization");
            }
            return;
        }

        // Convert SE3 pose (camera in world frame) to odometry
        Eigen::Matrix3f R = pose.rotationMatrix();
        Eigen::Vector3f t = pose.translation();

        // ORB-SLAM3 gives world-to-camera transform; invert for camera-in-world
        Eigen::Matrix3f R_inv = R.transpose();
        Eigen::Vector3f t_inv = -R_inv * t;

        Eigen::Quaternionf quat(R_inv);

        publish_odometry(msg->header.stamp, t_inv, quat);
        publish_tf(msg->header.stamp, t_inv, quat);
    }

    void publish_odometry(const builtin_interfaces::msg::Time& stamp,
                          const Eigen::Vector3f& position,
                          const Eigen::Quaternionf& orientation) {
        auto msg = nav_msgs::msg::Odometry();
        msg.header.stamp = stamp;
        msg.header.frame_id = odom_frame_;
        msg.child_frame_id = base_frame_;

        msg.pose.pose.position.x = position.x();
        msg.pose.pose.position.y = position.y();
        msg.pose.pose.position.z = position.z();
        msg.pose.pose.orientation.x = orientation.x();
        msg.pose.pose.orientation.y = orientation.y();
        msg.pose.pose.orientation.z = orientation.z();
        msg.pose.pose.orientation.w = orientation.w();

        msg.pose.covariance[0] = 0.01;   // x
        msg.pose.covariance[7] = 0.01;   // y
        msg.pose.covariance[14] = 0.01;  // z
        msg.pose.covariance[21] = 0.01;  // roll
        msg.pose.covariance[28] = 0.01;  // pitch
        msg.pose.covariance[35] = 0.01;  // yaw

        odom_pub_->publish(msg);
    }

    void publish_tf(const builtin_interfaces::msg::Time& stamp,
                    const Eigen::Vector3f& position,
                    const Eigen::Quaternionf& orientation) {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = stamp;
        tf_msg.header.frame_id = odom_frame_;
        tf_msg.child_frame_id = base_frame_;

        tf_msg.transform.translation.x = position.x();
        tf_msg.transform.translation.y = position.y();
        tf_msg.transform.translation.z = position.z();
        tf_msg.transform.rotation.x = orientation.x();
        tf_msg.transform.rotation.y = orientation.y();
        tf_msg.transform.rotation.z = orientation.z();
        tf_msg.transform.rotation.w = orientation.w();

        tf_broadcaster_->sendTransform(tf_msg);
    }

    void publish_map_points() {
        if (slam_system_->GetTrackingState() != 2) {
            return;
        }

        std::vector<ORB_SLAM3::MapPoint*> tracked = slam_system_->GetTrackedMapPoints();
        if (tracked.empty()) {
            return;
        }

        // Accumulate tracked points into persistent buffer
        for (auto* mp : tracked) {
            if (!mp || mp->isBad()) continue;
            Eigen::Vector3f pos = mp->GetWorldPos();
            accumulated_points_.push_back(pos);
        }

        // Cap accumulated buffer
        if (accumulated_points_.size() > MAX_POINTCLOUD_POINTS * 2) {
            size_t step = accumulated_points_.size() / MAX_POINTCLOUD_POINTS;
            std::vector<Eigen::Vector3f> sampled;
            sampled.reserve(MAX_POINTCLOUD_POINTS);
            for (size_t i = 0; i < accumulated_points_.size() && sampled.size() < MAX_POINTCLOUD_POINTS; i += step) {
                sampled.push_back(accumulated_points_[i]);
            }
            accumulated_points_ = std::move(sampled);
        }

        std::vector<Eigen::Vector3f>& valid_points = accumulated_points_;
        if (valid_points.empty()) {
            return;
        }

        // Build PointCloud2 message
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = now();
        cloud_msg.header.frame_id = odom_frame_;
        cloud_msg.height = 1;
        cloud_msg.width = valid_points.size();
        cloud_msg.is_dense = true;
        cloud_msg.is_bigendian = false;

        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(valid_points.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

        for (const auto& pt : valid_points) {
            *iter_x = pt.x();
            *iter_y = pt.y();
            *iter_z = pt.z();
            ++iter_x; ++iter_y; ++iter_z;
        }

        pointcloud_pub_->publish(cloud_msg);
    }

    std::unique_ptr<ORB_SLAM3::System> slam_system_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::TimerBase::SharedPtr pointcloud_timer_;

    std::vector<Eigen::Vector3f> accumulated_points_;

    std::string vocab_file_;
    std::string settings_file_;
    std::string odom_frame_;
    std::string base_frame_;
    std::string camera_frame_;
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OrbSlam3Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
