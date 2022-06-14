#include <pronto_msgs/msg/lidar_odometry_update.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/path.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

class LidarOdometryVisualizer {
public:
    LidarOdometryVisualizer(const rclcpp::Node::SharedPtr& node) {

        lidar_odom_sub_ = node->create_subscription<pronto_msgs::msg::LidarOdometryUpdate>("/aicp/transform_msg",
                                      100,
                                      std::bind(&LidarOdometryVisualizer::lidarOdomCallback,
                                      this, std::placeholders::_1));

        lidar_path_msg_.header.frame_id = "odom";

        lidar_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("/aicp/lidar_odometry_path", 100);

        init_pose_sub_ = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/state_estimator_pronto/pose",
                                        100,
                                        std::bind(&LidarOdometryVisualizer::initPoseCallback,
                                        this, std::placeholders::_1));



    }
    void lidarOdomCallback(const pronto_msgs::msg::LidarOdometryUpdate::SharedPtr msg){

        rel_transf_ = tf2::transformToEigen(msg->relative_transform);
        cumulative_transf_ = cumulative_transf_ * rel_transf_;

        cumulative_pose_.pose = tf2::toMsg(cumulative_transf_);
        cumulative_pose_.header.stamp = msg->curr_timestamp;
        lidar_path_msg_.header.stamp = msg->header.stamp;
        lidar_path_msg_.poses.push_back(cumulative_pose_);
        lidar_path_pub_->publish(lidar_path_msg_);
    }

    void initPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
        // take the very first one and shutdown
        tf2::fromMsg(msg->pose.pose, cumulative_transf_);
    }


private:
    Eigen::Isometry3d rel_transf_ = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d cumulative_transf_ = Eigen::Isometry3d::Identity();
    rclcpp::Subscription<pronto_msgs::msg::LidarOdometryUpdate>::SharedPtr lidar_odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr lidar_path_pub_;
    nav_msgs::msg::Path lidar_path_msg_;
    geometry_msgs::msg::PoseStamped cumulative_pose_;
};





int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    std::shared_ptr<rclcpp::Node> node;
    node = std::make_shared<rclcpp::Node>("lidar_odom_visualizer", node_options);
    LidarOdometryVisualizer viz(node);
    rclcpp::spin(node);
}
