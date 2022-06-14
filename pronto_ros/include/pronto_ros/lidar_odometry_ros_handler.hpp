#pragma once
#include <pronto_core/sensing_module.hpp>
#include <pronto_core/lidar_odometry_module.hpp>
#include <pronto_msgs/msg/lidar_odometry_update.hpp>
#include <rclcpp/rclcpp.hpp>

namespace pronto {

class LidarOdometryHandlerROS : public SensingModule<pronto_msgs::msg::LidarOdometryUpdate>{
public:
  LidarOdometryHandlerROS(const rclcpp::Node::SharedPtr& node);

  RBISUpdateInterface * processMessage(const pronto_msgs::msg::LidarOdometryUpdate* msg,
                                       StateEstimator* state_estimator);

  bool processMessageInit(const pronto_msgs::msg::LidarOdometryUpdate *msg,
                          const std::map<std::string, bool> &sensor_initialized,
                          const RBIS &default_state,
                          const RBIM &default_cov,
                          RBIS &init_state,
                          RBIM &init_cov);
private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<LidarOdometryModule> lidarodom_module_;
  LidarOdometryUpdate lidarodom_update_;
  int utime_offset_;
  bool use_measurement_cov_;
  LidarOdometryConfig cfg_;
};

}
