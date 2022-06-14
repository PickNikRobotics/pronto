#pragma once

#include <functional>
#include <sensor_msgs/msg/imu.hpp>
#include <pronto_core/ins_module.hpp>
#include <tf2_ros/transform_listener.h>

namespace pronto {
class InsHandlerROS : public SensingModule<sensor_msgs::msg::Imu> {
public:
    InsHandlerROS(const std::shared_ptr<rclcpp::Node> &node);

    RBISUpdateInterface* processMessage(const sensor_msgs::msg::Imu *imu_msg, StateEstimator *est) override;

    bool processMessageInit(const sensor_msgs::msg::Imu *imu_msg,
                            const std::map<std::string, bool> &sensor_initialized,
                            const RBIS &default_state,
                            const RBIM &default_cov,
                            RBIS &init_state,
                            RBIM &init_cov) override;


private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<tf2_ros::Buffer> tf_imu_to_body_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_imu_to_body_listener_;
    ImuMeasurement imu_meas_;
    InsModule ins_module_;
    uint64_t counter = 0;
    uint16_t downsample_factor_ = 1;
    std::string imu_topic_ = "/sensors/imu";
    bool roll_forward_on_receive_ = true;
    int64_t utime_offset_ = 0;
};

}  // namespace pronto
