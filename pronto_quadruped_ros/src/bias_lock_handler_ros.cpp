/* Copyright (c) 2019-2020 University of Oxford
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

#include "pronto_quadruped_ros/bias_lock_handler_ros.hpp"

using namespace pronto;
using namespace pronto::quadruped;

ImuBiasLockROS::ImuBiasLockROS(const rclcpp::Node::SharedPtr& node) : ImuBiasLockBaseROS(node)
{
}

RBISUpdateInterface* ImuBiasLockROS::processMessage(const sensor_msgs::msg::Imu *msg,
                                                    StateEstimator *est)
{
  msgToImuMeasurement(*msg, bias_lock_imu_msg_);
  RBISUpdateInterface* update = bias_lock_module_->processMessage(&bias_lock_imu_msg_, est);

  if(update != nullptr){
    RBIS head_state;
    RBIM head_cov;
    est->getHeadState(head_state, head_cov);
    RCLCPP_INFO_STREAM(node_->get_logger(), "Bias update. Prior accel bias: " << head_state.accelBias().transpose() << std::endl
                              <<  "Prior gyro bias: " << head_state.gyroBias().transpose());
  }
  return update;
}

bool ImuBiasLockROS::processMessageInit(const sensor_msgs::msg::Imu *msg,
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

void ImuBiasLockROS::processSecondaryMessage(const sensor_msgs::msg::JointState &msg) {
  jointStateFromROS(msg, bias_lock_js_msg_);
  bias_lock_module_->processSecondaryMessage(bias_lock_js_msg_);
}

// pronto_msgs/JointStateWithAcceleration (includes acceleration)
void ImuBiasLockWithAccelerationROS::processSecondaryMessage(const pronto_msgs::msg::JointStateWithAcceleration &msg) {
  jointStateWithAccelerationFromROS(msg, bias_lock_js_msg_);
  bias_lock_module_->processSecondaryMessage(bias_lock_js_msg_);
}
