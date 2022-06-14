#pragma once
#include <rclcpp/rclcpp.hpp>
#include <pronto_core/vicon_module.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_eigen/tf2_eigen.h>

namespace pronto {

class ViconHandlerROS : public SensingModule<geometry_msgs::msg::TransformStamped> {
public:
    ViconHandlerROS(rclcpp::Node::SharedPtr& node);

    RBISUpdateInterface* processMessage(const geometry_msgs::msg::TransformStamped *msg,
                                        StateEstimator *est);

    bool processMessageInit(const geometry_msgs::msg::TransformStamped *msg,
                            const std::map<std::string, bool> &sensor_initialized,
                            const RBIS &default_state,
                            const RBIM &default_cov,
                            RBIS &init_state,
                            RBIM &init_cov);


private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<ViconModule> vicon_module_;
    RigidTransform vicon_transf_;
};

} // namespace pronto
