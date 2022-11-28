#include "pronto_quadruped_ros/conversions.hpp"

#include <iterator>
#include "rclcpp/rclcpp.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pronto_quadruper_ros_conversions");

namespace pronto {
namespace quadruped {

bool jointStateFromROS(const sensor_msgs::msg::JointState& msg,
                       uint64_t& utime,
                       JointState& q,
                       JointState& qd,
                       JointState& qdd,
                       JointState& tau)
{
    // store message time in microseconds
    utime = msg.header.stamp.sec * 1e6 + msg.header.stamp.nanosec / 1000;

    for (int i = 0; i < orderedJointNames.size(); i++) {
      const auto& joint_name = orderedJointNames[i];
      auto it = std::find(msg.name.begin(), msg.name.end(), joint_name);
      if (it == msg.name.end()) {
        RCLCPP_WARN_STREAM(LOGGER, "Joint State expected to contain " << joint_name);
        return false;
      }

      int pos = std::distance(msg.name.begin(), it);
      q(i) = msg.position[pos];
      qd(i) = msg.velocity[pos];
      tau(i) = msg.effort[pos];
    }

    qdd.setZero(); // TODO compute the acceleration

    return true;
}

bool jointStateWithAccelerationFromROS(const pronto_msgs::msg::JointStateWithAcceleration& msg,
                               uint64_t& utime,
                               JointState& q,
                               JointState& qd,
                               JointState& qdd,
                               JointState& tau)
{
    // store message time in microseconds
    utime = msg.header.stamp.sec * 1e6 + msg.header.stamp.nanosec / 1000;

    for (int i = 0; i < orderedJointNames.size(); i++) {
      const auto& joint_name = orderedJointNames[i];
      auto it = std::find(msg.name.begin(), msg.name.end(), joint_name);
      if (it == msg.name.end()) {
        RCLCPP_WARN_STREAM(LOGGER, "Joint State expected to contain " << joint_name);
        return false;
      }

      int pos = std::distance(msg.name.begin(), it);
      q(i) = msg.position[pos];
      qd(i) = msg.velocity[pos];
      qdd(i) = msg.acceleration[pos];
      tau(i) = msg.effort[pos];
    }

    qdd.setZero(); // TODO compute the acceleration

    return true;
}

}  // namespace quadruped
}  // namespace pronto
