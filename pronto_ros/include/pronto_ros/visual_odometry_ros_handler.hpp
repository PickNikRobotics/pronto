#pragma once
#include <pronto_core/sensing_module.hpp>
#include <pronto_core/visual_odometry_module.hpp>
#include <pronto_msgs/msg/visual_odometry_update.hpp>
#include <rclcpp/rclcpp.hpp>

namespace pronto {

class VisualOdometryHandlerROS : public SensingModule<pronto_msgs::msg::VisualOdometryUpdate>{
public:
  VisualOdometryHandlerROS(const rclcpp::Node::SharedPtr& node);

  RBISUpdateInterface * processMessage(const pronto_msgs::msg::VisualOdometryUpdate* msg,
                                       StateEstimator* state_estimator);

  bool processMessageInit(const pronto_msgs::msg::VisualOdometryUpdate *msg,
                          const std::map<std::string, bool> &sensor_initialized,
                          const RBIS &default_state,
                          const RBIM &default_cov,
                          RBIS &init_state,
                          RBIM &init_cov);
private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<VisualOdometryModule> vo_module_;
  VisualOdometryUpdate vo_update_;
  int utime_offset_;
  // ros::Duration msg_time_offset_;
};

}
