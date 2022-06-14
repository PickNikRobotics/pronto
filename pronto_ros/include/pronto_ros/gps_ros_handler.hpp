#pragma once

#include <pronto_core/gps_module.hpp>
#include <pronto_core/sensing_module.hpp>
#include <pronto_msgs/msg/gps_data.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pronto_ros/pronto_ros_conversions.hpp>

namespace pronto {
class GPSHandlerROS : public SensingModule<pronto_msgs::msg::GPSData> {
public:
    GPSHandlerROS(const rclcpp::Node::SharedPtr& node);
    RBISUpdateInterface* processMessage(const pronto_msgs::msg::GPSData *msg,
                                        StateEstimator *est) override;

    bool processMessageInit(const pronto_msgs::msg::GPSData *msg,
                            const std::map<std::string, bool> &sensor_initialized,
                            const RBIS &default_state,
                            const RBIM &default_cov,
                            RBIS &init_state,
                            RBIM &init_cov) override;

protected:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<GPSModule> gps_module_;
    GPSMeasurement gps_meas_;
};
} // namespace pronto
