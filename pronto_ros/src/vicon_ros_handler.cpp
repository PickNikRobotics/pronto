#include "pronto_ros/vicon_ros_handler.hpp"
#include "pronto_ros/pronto_ros_conversions.hpp"
#include <tf2_ros/transform_listener.h>

namespace pronto {

ViconHandlerROS::ViconHandlerROS(rclcpp::Node::SharedPtr &node) :
node_(node)
{
    std::string prefix = "vicon.";
    ViconConfig cfg;
    std::string mode_str;
    node_->get_parameter(prefix + "mode", mode_str);
    if (mode_str.compare("position") == 0) {
      cfg.mode = ViconMode::MODE_POSITION;
      std::cout << "Vicon will provide position measurements." << std::endl;
    }
    else if (mode_str.compare("position_orient") == 0) {
      cfg.mode = ViconMode::MODE_POSITION_ORIENT;
      std::cout << "Vicon will provide position and orientation measurements." << std::endl;
    }
    else if (mode_str.compare("orientation") == 0) {
      cfg.mode = ViconMode::MODE_ORIENTATION;
      std::cout << "Vicon will provide orientation measurements." << std::endl;
    }
    else if (mode_str.compare("yaw") == 0) {
      cfg.mode = ViconMode::MODE_YAW;
      std::cout << "Vicon will provide yaw orientation measurements." << std::endl;
    }
    else {
      cfg.mode = ViconMode::MODE_POSITION;
      std::cout << "Unrecognized Vicon mode. Using position mode by default." << std::endl;
    }

    bool apply_frame = false;
    std::string frame_from;
    std::string frame_to;
    if(node_->get_parameter(prefix + "apply_frame", apply_frame) && apply_frame){
        if(node_->get_parameter(prefix + "frame_from", frame_from)){
            if(node_->get_parameter(prefix + "frame_to", frame_to)){
                std::shared_ptr<tf2_ros::TransformListener> listener;
                auto tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
                geometry_msgs::msg::TransformStamped transform;
                try {
                    auto trans_msg = tf_buffer_->lookupTransform(frame_to, frame_from, rclcpp::Time(0));
                    cfg.body_to_vicon = tf2::transformToEigen(transform);
                } catch (tf2::TransformException ex) {
                    RCLCPP_ERROR(node_->get_logger(), "%s",ex.what());
                }

            }
        }

    } else {
        cfg.body_to_vicon.setIdentity();
    }

    if(!node_->get_parameter(prefix + "r_xyz", cfg.r_vicon_xyz)){
        cfg.r_vicon_xyz = 0;
        RCLCPP_WARN(node_->get_logger(), "Couldn't get param \"r_xyz\". Setting to zero.");
    }

    if(!node_->get_parameter(prefix + "r_chi", cfg.r_vicon_chi)){
        cfg.r_vicon_chi = 0;
        RCLCPP_WARN(node_->get_logger(), "Couldn't get param \"r_chi\". Setting to zero.");
    }
    vicon_module_.reset(new ViconModule(cfg));
}

RBISUpdateInterface* ViconHandlerROS::processMessage(const geometry_msgs::msg::TransformStamped *msg,
                                                     StateEstimator *est)
{
    rigidTransformFromROS(*msg, vicon_transf_);
    return vicon_module_->processMessage(&vicon_transf_,est);
}

bool ViconHandlerROS::processMessageInit(const geometry_msgs::msg::TransformStamped *msg,
                                         const std::map<std::string, bool> &sensor_initialized,
                                         const RBIS &default_state,
                                         const RBIM &default_cov,
                                         RBIS &init_state,
                                         RBIM &init_cov){
    rigidTransformFromROS(*msg, vicon_transf_);
    return vicon_module_->processMessageInit(&vicon_transf_,
                                             sensor_initialized,
                                             default_state,
                                             default_cov,
                                             init_state,
                                             init_cov);
}
} // namespace pronto
