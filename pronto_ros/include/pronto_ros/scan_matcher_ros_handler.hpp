#pragma once
#include <Eigen/Dense>
#include <pronto_core/rbis_update_interface.hpp>
#include <pronto_core/state_est.hpp>
#include <pronto_core/scan_matcher_module.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace pronto {

class ScanMatcherHandler : public SensingModule<geometry_msgs::msg::PoseWithCovarianceStamped>{
public:
  ScanMatcherHandler(const rclcpp::Node::SharedPtr& node);

  RBISUpdateInterface * processMessage(const geometry_msgs::msg::PoseWithCovarianceStamped* msg,
                                       StateEstimator* state_estimator) override;

  bool processMessageInit(const geometry_msgs::msg::PoseWithCovarianceStamped *msg,
                          const std::map<std::string, bool> &sensor_initialized,
                          const RBIS &default_state,
                          const RBIM &default_cov,
                          RBIS &init_state,
                          RBIM &init_cov) override;

protected:
  ScanMatcherModule scan_matcher_module_;
  PoseMeasurement pose_meas_;
  std::shared_ptr<rclcpp::Node> node_;
  tf2::Quaternion tf_q;
};


} // namespace pronto
