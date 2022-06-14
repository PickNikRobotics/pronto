#pragma once
#include <rclcpp/rclcpp.hpp>
#include <pronto_core/pose_meas_module.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

namespace pronto {

class PoseHandlerROS : public SensingModule<geometry_msgs::msg::PoseWithCovarianceStamped> {
public:
    PoseHandlerROS(const rclcpp::Node::SharedPtr& node);

    RBISUpdateInterface* processMessage(const geometry_msgs::msg::PoseWithCovarianceStamped *msg,
                                        StateEstimator *est);

    bool processMessageInit(const geometry_msgs::msg::PoseWithCovarianceStamped *msg,
                            const std::map<std::string, bool> &sensor_initialized,
                            const RBIS &default_state,
                            const RBIM &default_cov,
                            RBIS &init_state,
                            RBIM &init_cov);
private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<PoseMeasModule> pose_module_;
    PoseMeasurement pose_meas_;
};

}
