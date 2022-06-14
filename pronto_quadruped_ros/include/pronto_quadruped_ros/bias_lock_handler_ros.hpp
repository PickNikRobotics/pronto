/* Copyright (c) 2019 University of Oxford
 * All rights reserved.
 *
 * Author: Marco Camurri (mcamurri@robots.ox.ac.uk)
 *
 * This file is part of pronto_quadruped,
 * a library for leg odometry on quadruped robots.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#pragma once
#include <memory>
#include <pronto_core/rigidbody.hpp>  // for g_val
#include <pronto_quadruped/ImuBiasLock.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <pronto_ros/pronto_ros_conversions.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <pronto_msgs/msg/joint_state_with_acceleration.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>

namespace pronto {
namespace quadruped {

template <class JointStateT>
class ImuBiasLockBaseROS : public DualSensingModule<sensor_msgs::msg::Imu, JointStateT>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
  ImuBiasLockBaseROS() = delete;
  ImuBiasLockBaseROS(const rclcpp::Node::SharedPtr& node);
  RBISUpdateInterface* processMessage(const sensor_msgs::msg::Imu *msg,
                                      StateEstimator *est) override;

  bool processMessageInit(const sensor_msgs::msg::Imu *msg,
                          const std::map<std::string, bool> &sensor_initialized,
                          const RBIS &default_state,
                          const RBIM &default_cov, RBIS &init_state,
                          RBIM &init_cov) override;

protected:
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<quadruped::ImuBiasLock> bias_lock_module_;
  pronto::JointState bias_lock_js_msg_;
  pronto::ImuMeasurement bias_lock_imu_msg_;
  visualization_msgs::msg::Marker imu_arrow_;
  visualization_msgs::msg::Marker base_arrow_;
  visualization_msgs::msg::Marker base_more_arrow_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr base_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr base_more_marker_pub_;
  visualization_msgs::msg::MarkerArray frame_markers_;
  visualization_msgs::msg::Marker axis_marker_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  bool publish_debug_topics_ = true;
  bool publish_transforms_ = true;
};

template <class JointStateT>
ImuBiasLockBaseROS<JointStateT>::ImuBiasLockBaseROS(const rclcpp::Node::SharedPtr& node) : node_(node)
{
  std::shared_ptr<tf2_ros::Buffer> tfBuffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_imu_to_body_listener_;
  tfBuffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_imu_to_body_listener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
  broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);

  std::string ins_param_prefix = "ins.";
  std::string lock_param_prefix = "bias_lock.";

  std::string imu_frame = "imu_link";

  node_->get_parameter(ins_param_prefix + "frame", imu_frame);
  std::string base_frame;
  // Get base_frame
  if (!node_->has_parameter("base_link_name"))
  {
    node_->declare_parameter("base_link_name", "base");
  }
  base_frame = node_->get_parameter("base_link_name").as_string();
  RCLCPP_INFO_STREAM(node_->get_logger(), "[ImuBiasLockBaseROS] Name of base_link: '" << base_frame << "'");
  Eigen::Isometry3d ins_to_body = Eigen::Isometry3d::Identity();
  while(rclcpp::ok()) {
    try {
      // lookupTransform API is : target_frame, source_frame
      geometry_msgs::msg::TransformStamped temp_transform = tfBuffer->lookupTransform(base_frame, imu_frame, rclcpp::Time(0));
      ins_to_body = tf2::transformToEigen(temp_transform.transform);
      RCLCPP_INFO_STREAM(node_->get_logger(), "IMU (" << imu_frame <<") to base (" << base_frame << ") transform: translation=(" << ins_to_body.translation().transpose() << "), rotation=(" << ins_to_body.rotation() << ")");
      break;
    }
    catch (const tf2::TransformException& ex) {
      RCLCPP_ERROR(node_->get_logger(), "%s", ex.what());
      rclcpp::sleep_for(std::chrono::milliseconds(1000));
    }
  }

  quadruped::ImuBiasLockConfig cfg;
  node_->get_parameter(lock_param_prefix + "torque_threshold", cfg.torque_threshold_);
  node_->get_parameter(lock_param_prefix + "velocity_threshold", cfg.velocity_threshold_);
  cfg.verbose_ = node_->declare_parameter<bool>(lock_param_prefix + "verbose", false);

  if(!node_->get_parameter(ins_param_prefix + "timestep_dt", cfg.dt_)){
    RCLCPP_WARN_STREAM(node_->get_logger(), "Couldn't read dt. Using default: " << cfg.dt_);
  }

  // Determine what to publish:
  publish_debug_topics_ = node_->declare_parameter<bool>(lock_param_prefix + "publish_debug_topics", true);  // Default to 'true' to preserve previous behaviour
  publish_transforms_ = node_->declare_parameter<bool>(lock_param_prefix + "publish_transforms", true);    // Default to 'true' to preserve previous behaviour
  RCLCPP_INFO_STREAM(node_->get_logger(), "[ImuBiasLockBaseROS] Publishing debug topics:  " << std::boolalpha << publish_debug_topics_ << "\n" <<
                    "                   Publishing transforms:    " << publish_transforms_);

  if (publish_debug_topics_) {
    status_pub_ = node_->create_publisher<geometry_msgs::msg::PointStamped>("/state_estimator_pronto/recording_bias", 100);
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("/state_estimator_pronto/imu_arrows", 100);
    base_marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("/state_estimator_pronto/base_arrow", 100);
    base_more_marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("/state_estimator_pronto/base_more_arrow", 100);
    imu_arrow_.color.a = 1;
    imu_arrow_.color.r = 1;
    imu_arrow_.color.g = 0;
    imu_arrow_.color.b = 0;

    imu_arrow_.scale.x = 0.05;
    imu_arrow_.scale.y = 0.07;
    imu_arrow_.scale.z = 0.1;

    imu_arrow_.type = visualization_msgs::msg::Marker::ARROW;

    imu_arrow_.action = visualization_msgs::msg::Marker::ADD;
    imu_arrow_.header.frame_id = imu_frame;
    imu_arrow_.points.resize(2);
    imu_arrow_.points[0].x = 0;
    imu_arrow_.points[0].y = 0;
    imu_arrow_.points[0].z = 0;

    base_arrow_ = imu_arrow_;
    base_arrow_.header.frame_id = base_frame;
    base_arrow_.color.r = 0;
    base_arrow_.color.g = 1;
  }

  bias_lock_module_.reset(new quadruped::ImuBiasLock(ins_to_body, cfg));
}

template <class JointStateT>
RBISUpdateInterface* ImuBiasLockBaseROS<JointStateT>::processMessage(const sensor_msgs::msg::Imu *msg,
                                                                    StateEstimator *est)
{
  msgToImuMeasurement(*msg, bias_lock_imu_msg_);

  if (publish_debug_topics_) {
    imu_arrow_.header.stamp = msg->header.stamp;

    imu_arrow_.points[1].x = 0.1*msg->linear_acceleration.x;
    imu_arrow_.points[1].y = 0.1*msg->linear_acceleration.y;
    imu_arrow_.points[1].z = 0.1*msg->linear_acceleration.z;

    marker_pub_->publish(imu_arrow_);

    // const Eigen::Vector3d& accel_ = bias_lock_module_->getCurrentAccel();
    // Eigen::Vector3d accel_corrected_ = bias_lock_module_->getCurrentCorrectedAccel();
    base_arrow_.points.resize(2);
    base_arrow_.points[0].x = 0;
    base_arrow_.points[0].y = 0;
    base_arrow_.points[0].z = 0;

    // Gravity vector (note, g_vec is in ENU notation so need to take the negative)
    base_arrow_.points[1].x = 0.1 * -g_vec.x();
    base_arrow_.points[1].y = 0.1 * -g_vec.y();
    base_arrow_.points[1].z = 0.1 * -g_vec.z();

    base_marker_pub_->publish(base_arrow_);

    base_more_arrow_ = base_arrow_;
    const Eigen::Vector3d& bias = bias_lock_module_->getCurrentProperAccelBias();
    base_more_arrow_.points.resize(2);
    base_more_arrow_.points[0].x = 0;
    base_more_arrow_.points[0].y = 0;
    base_more_arrow_.points[0].z = 0;

    base_more_arrow_.points[1].x = 0.1 * bias.x();
    base_more_arrow_.points[1].y = 0.1 * bias.y();
    base_more_arrow_.points[1].z = 0.1 * bias.z();

    base_more_arrow_.color.r = 0;
    base_more_arrow_.color.g = 0;
    base_more_arrow_.color.b = 1;

    base_more_marker_pub_->publish(base_more_arrow_);

    geometry_msgs::msg::PointStamped state_msg;
    state_msg.header.stamp = msg->header.stamp;
    state_msg.point.x = int(bias_lock_module_->getRecordStatus());
    state_msg.point.y = state_msg.point.x;
    state_msg.point.z = state_msg.point.x;
    status_pub_->publish(state_msg);
  }

  if (publish_transforms_) {
    const Eigen::Isometry3d& gravity_transform = bias_lock_module_->getGravityTransform();
    const Eigen::Isometry3d& bias_transform = bias_lock_module_->getBiasTransform();

    geometry_msgs::msg::TransformStamped msg_temp = tf2::eigenToTransform(gravity_transform);
    msg_temp.child_frame_id = "gravity";
    msg_temp.header.frame_id = base_arrow_.header.frame_id;
    msg_temp.header.stamp = msg->header.stamp;

    broadcaster_->sendTransform(msg_temp);

    msg_temp = tf2::eigenToTransform(bias_transform);
    msg_temp.child_frame_id = "bias";
    msg_temp.header.frame_id = base_arrow_.header.frame_id;
    msg_temp.header.stamp = msg->header.stamp;

    broadcaster_->sendTransform(msg_temp);
  }

  return bias_lock_module_->processMessage(&bias_lock_imu_msg_, est);
}

template <class JointStateT>
bool ImuBiasLockBaseROS<JointStateT>::processMessageInit(const sensor_msgs::msg::Imu *msg,
                                                        const std::map<std::string, bool> &sensor_initialized,
                                                        const RBIS &default_state,
                                                        const RBIM &default_cov,
                                                        RBIS &init_state,
                                                        RBIM &init_cov)
{
  msgToImuMeasurement(*msg, bias_lock_imu_msg_);
  return bias_lock_module_->processMessageInit(&bias_lock_imu_msg_,
                                               sensor_initialized,
                                               default_state,
                                               default_cov,
                                               init_state,
                                               init_cov);
}



class ImuBiasLockROS : public ImuBiasLockBaseROS<sensor_msgs::msg::JointState>
{
public:
    ImuBiasLockROS(const rclcpp::Node::SharedPtr& node);
    virtual ~ImuBiasLockROS() = default;


    RBISUpdateInterface* processMessage(const sensor_msgs::msg::Imu *msg,
                                        StateEstimator *est) override;

    bool processMessageInit(const sensor_msgs::msg::Imu *msg,
                            const std::map<std::string, bool> &sensor_initialized,
                            const RBIS &default_state,
                            const RBIM &default_cov, RBIS &init_state,
                            RBIM &init_cov) override;


    void processSecondaryMessage(const sensor_msgs::msg::JointState& msg) override;
};

class ImuBiasLockWithAccelerationROS : public ImuBiasLockBaseROS<pronto_msgs::msg::JointStateWithAcceleration>
{
public:
    ImuBiasLockWithAccelerationROS(const rclcpp::Node::SharedPtr& node) : ImuBiasLockBaseROS<pronto_msgs::msg::JointStateWithAcceleration>(node) {}
    virtual ~ImuBiasLockWithAccelerationROS() = default;

    void processSecondaryMessage(const pronto_msgs::msg::JointStateWithAcceleration& msg) override;
};

}  // namespace quadruped
}  // namespace pronto
