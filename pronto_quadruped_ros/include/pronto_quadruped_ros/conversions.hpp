#pragma once

#include <sensor_msgs/msg/joint_state.hpp>
#include <pronto_msgs/msg/joint_state_with_acceleration.hpp>
#include <pronto_quadruped_commons/declarations.h>

namespace pronto {
namespace quadruped {
bool jointStateFromROS(const sensor_msgs::msg::JointState& msg,
                       uint64_t& utime,
                       JointState& q,
                       JointState& qd,
                       JointState& qdd,
                       JointState& tau);

bool jointStateWithAccelerationFromROS(const pronto_msgs::msg::JointStateWithAcceleration& msg,
                               uint64_t& utime,
                               JointState& q,
                               JointState& qd,
                               JointState& qdd,
                               JointState& tau);
}  // namespace quadruped
}  // namespace pronto
