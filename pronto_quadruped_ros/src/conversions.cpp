#include "pronto_quadruped_ros/conversions.hpp"
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
    // if the size of the joint state message does not match our own,
    // we silently return an invalid update
    if (static_cast<int>(static_cast<const sensor_msgs::msg::JointState&>(msg).position.size()) != q.size() ||
        static_cast<int>(static_cast<const sensor_msgs::msg::JointState&>(msg).velocity.size()) != q.size() ||
        static_cast<int>(static_cast<const sensor_msgs::msg::JointState&>(msg).effort.size()) != q.size()){
        RCLCPP_WARN_STREAM(LOGGER, "Joint State is expected " << \
                                 q.size() << " joints but "\
                                 << msg.position.size() << " / " << msg.velocity.size() << " / " << msg.effort.size()
                                 << " are provided.");
        return false;
    }
    // store message time in microseconds
    utime = msg.header.stamp.sec * 1e6 + msg.header.stamp.nanosec / 1000;
    for(int i=0; i<12; i++){
      q(i) = msg.position[i];
      qd(i) = msg.velocity[i];
      tau(i) = msg.effort[i];
    }
    //q = Eigen::Map<const JointState>(msg.position.data());
    //qd = Eigen::Map<const JointState>(msg.velocity.data());
    //tau = Eigen::Map<const JointState>(msg.effort.data());

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
    // if the size of the joint state message does not match our own,
    // we silently return an invalid update
    if (static_cast<int>(static_cast<const pronto_msgs::msg::JointStateWithAcceleration&>(msg).position.size()) != q.size() ||
        static_cast<int>(static_cast<const pronto_msgs::msg::JointStateWithAcceleration&>(msg).velocity.size()) != q.size() ||
        static_cast<int>(static_cast<const pronto_msgs::msg::JointStateWithAcceleration&>(msg).acceleration.size()) != q.size() ||
        static_cast<int>(static_cast<const pronto_msgs::msg::JointStateWithAcceleration&>(msg).effort.size()) != q.size()){
        RCLCPP_WARN_STREAM(LOGGER, "Joint State is expected " << \
                                 q.size() << " joints but "\
                                 << msg.position.size() << " / " << msg.velocity.size() << " / " << msg.acceleration.size() << " / " << msg.effort.size()
                                 << " are provided.");
        return false;
    }
    // store message time in microseconds
    utime = msg.header.stamp.sec * 1e6 + 1e-3 * msg.header.stamp.nanosec;
    for(int i=0; i<12; i++){
      q(i) = msg.position[i];
      qd(i) = msg.velocity[i];
      qdd(i) = msg.acceleration[i];
      tau(i) = msg.effort[i];
    }

    return true;
}

}  // namespace quadruped
}  // namespace pronto
