#include "pronto_ros/ros_frontend.hpp"

namespace pronto {
using SensorId = pronto::ROSFrontEnd::SensorId;

ROSFrontEnd::ROSFrontEnd(const std::shared_ptr<rclcpp::Node> &node, bool verbose) :
    node_(node), verbose_(verbose)
{
    bool publish_pose;
    // looking for relative param names, the nodehandle namespace will be added
    // automatically, e.g. publish_pose -> /state_estimator_pronto/publish_pose
    if(node_->get_parameter("publish_pose", publish_pose)){
        if(publish_pose){
            std::string pose_frame_id = "odom";
            if(!node_->get_parameter("pose_frame_id", pose_frame_id)){
                RCLCPP_WARN_STREAM(LOGGER, "Couldn't get param \"pose_frame_id\". Setting default to: \"" << pose_frame_id <<"\"");
            }
            std::string pose_topic = "POSE_BODY";
            if(node_->get_parameter("pose_topic", pose_topic)){
                pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topic, 200);
                pose_msg_.header.frame_id = pose_frame_id;
            }
            std::string twist_topic = "TWIST_BODY";
            std::string twist_frame_id = "base";
            if(!node_->get_parameter("twist_frame_id", twist_frame_id)){
                RCLCPP_WARN_STREAM(LOGGER, "Couldn't get param \"twist_frame_id\". Setting default to: \"" << twist_frame_id <<"\"");
            }
            if(node_->get_parameter("twist_topic", twist_topic)){
                twist_pub_ = node_->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(twist_topic, 200);
                twist_msg_.header.frame_id = twist_frame_id;
            }
            // try to
            if(!node_->get_parameter("publish_tf", publish_tf_)){
                RCLCPP_WARN(LOGGER, "Couldn't get param \"publish_tf\". Not publishing TF.");
            }
            if(publish_tf_){
                std::string tf_child_frame_id = "base";
                if(!node_->get_parameter("tf_child_frame_id", tf_child_frame_id)){
                    RCLCPP_WARN(LOGGER, "Couldn't get param \"tf_frame_id\". Setting default to \"base\".");
                }
                transform_msg_.header.frame_id = pose_frame_id;
                // NOTE implicitly assuming the twist frame id is the base frame
                transform_msg_.child_frame_id = tf_child_frame_id;
                RCLCPP_INFO_STREAM(LOGGER, "Publishing TF with frame_id: \"" << transform_msg_.header.frame_id << "\" and child_frame_id: \"" << transform_msg_.child_frame_id << "\"");
                tf2_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
            }
        }

    }

    double history_span = 1e6; // keep 1 second as default
    if(!node_->get_parameter("utime_history_span", history_span)){
        RCLCPP_WARN_STREAM(LOGGER, "Couldn't get param \"utime_history_span\". Setting default to \"" << history_span << "\"");
    }
    history_span_ = history_span;
    initializeState();
    initializeCovariance();
    if(verbose_){
        RCLCPP_INFO_STREAM(LOGGER, "Frontend constructed.");
    }

    aicp_path_publisher_ = node_->create_publisher<nav_msgs::msg::Path>("/aicp/relative_path", 100);
}

void ROSFrontEnd::initializeState()
{
    // setting the initial values from the state
    std::vector<double> init_velocity = std::vector<double>(3,0.0);
    if(!node_->get_parameter("x0.velocity", init_velocity)){
        RCLCPP_WARN_STREAM(LOGGER, "Couldn't get default velocity. Setting to zero.");
    }
    default_state.velocity() = Eigen::Map<Eigen::Vector3d>(init_velocity.data());

    // setting the initial values from the state
    std::vector<double> init_position  = std::vector<double>(3,0.0);
    if(!node_->get_parameter("x0.position", init_position)){
        RCLCPP_WARN_STREAM(LOGGER, "Couldn't get default position. Setting to zero.");
    }
    default_state.position() = Eigen::Map<Eigen::Vector3d>(init_position.data());

    std::vector<double> init_omega = std::vector<double>(3,0.0);
    if(!node_->get_parameter("x0.angular_velocity", init_omega)){
        RCLCPP_WARN_STREAM(LOGGER, "Couldn't get default ang velocity. Setting to zero.");
    }
    default_state.angularVelocity() = Eigen::Map<Eigen::Vector3d>(init_omega.data());

    std::vector<double> init_orient = std::vector<double>(3,0.0);
    if(!node_->get_parameter("x0.rpy", init_orient)){
        RCLCPP_WARN_STREAM(LOGGER, "Couldn't get default orientation. Setting to zero.");
        default_state.orientation() = Eigen::Quaterniond::Identity();
    }
    default_state.orientation() = rotation::setQuatEulerAngles(Eigen::Map<Eigen::Vector3d>(init_orient.data()));
    init_state = default_state;
    head_state = default_state;
    if(verbose_){
        RCLCPP_INFO_STREAM(LOGGER, "Filter Initial State initialized.");
    }
}


void ROSFrontEnd::initializeCovariance(){
    default_cov = RBIM::Zero();

    double sigma_Delta_xy_init = 0;
    if(!node_->get_parameter("sigma0.Delta_xy", sigma_Delta_xy_init)){
        RCLCPP_WARN_STREAM(LOGGER, "Couldn't get param sigma0.Delta_xy. Setting to zero.");
    }


    double sigma_Delta_z_init = 0;
    if(!node_->get_parameter("sigma0.Delta_z", sigma_Delta_z_init)){
        RCLCPP_WARN_STREAM(LOGGER, "Couldn't get param sigma0.Delta_z. Setting to zero.");
    }

    double sigma_chi_xy_init = 0;
    if(!node_->get_parameter("sigma0.chi_xy", sigma_chi_xy_init)){
        RCLCPP_WARN_STREAM(LOGGER, "Couldn't get param sigma0.chi_xy. Setting to zero.");
    }
    sigma_chi_xy_init *= M_PI / 180.0; // convert to radians

    double sigma_chi_z_init = 0;
    if(!node_->get_parameter("sigma0.chi_z", sigma_chi_z_init)){
        RCLCPP_WARN_STREAM(LOGGER, "Couldn't get param sigma0.chi_z. Setting to zero.");
    }
    sigma_chi_z_init *= M_PI / 180.0; // convert to radians

    double sigma_vb_init = 0;
    if(!node_->get_parameter("sigma0.vb", sigma_vb_init)){
        RCLCPP_WARN_STREAM(LOGGER, "Couldn't get param sigma0.vb. Setting to zero.");
    }

    double sigma_gyro_bias_init = 0;
    if(!node_->get_parameter("sigma0.gyro_bias", sigma_gyro_bias_init)){
        RCLCPP_WARN_STREAM(LOGGER, "Couldn't get param sigma0.gyro_bias. Setting to zero.");
    }
    sigma_gyro_bias_init *= M_PI / 180.0; // convert to radians

    double sigma_accelerometer_bias_init = 0;
    if(!node_->get_parameter("sigma0.accel_bias", sigma_accelerometer_bias_init)){
        RCLCPP_WARN_STREAM(LOGGER, "Couldn't get param sigma0.accel_bias. Setting to zero.");
    }

    Eigen::Vector3d xyz_cov_diag =
        Eigen::Array3d(sigma_Delta_xy_init, sigma_Delta_xy_init, sigma_Delta_z_init).square();

    Eigen::Vector3d init_chi_cov_diag = Eigen::Array3d(sigma_chi_xy_init, sigma_chi_xy_init, sigma_chi_z_init).square();

    //set all the sub-blocks of the covariance matrix
    default_cov.block<3, 3>(RBIS::velocity_ind, RBIS::velocity_ind) = std::pow(sigma_vb_init,2) * Eigen::Matrix3d::Identity();
    default_cov.block<3, 3>(RBIS::chi_ind, RBIS::chi_ind) = init_chi_cov_diag.asDiagonal();
    default_cov.block<3, 3>(RBIS::position_ind, RBIS::position_ind) = xyz_cov_diag.asDiagonal();
    default_cov.block<3, 3>(RBIS::gyro_bias_ind, RBIS::gyro_bias_ind) = Eigen::Matrix3d::Identity()
        * std::pow(sigma_gyro_bias_init,2);
    default_cov.block<3, 3>(RBIS::accel_bias_ind, RBIS::accel_bias_ind) = Eigen::Matrix3d::Identity()
        * std::pow(sigma_accelerometer_bias_init,2);

    init_cov = default_cov;
    head_cov = default_cov;
    if(verbose_){
        RCLCPP_INFO_STREAM(LOGGER, "Filter Covariance initialized.");
    }
}

ROSFrontEnd::~ROSFrontEnd()
{
}

bool ROSFrontEnd::initializeFilter()
{
    // if the modules are not ready we return false
    if(!areModulesInitialized())
    {
        return false;
    }
    // if the filter is already initialized we quietly return
    if(isFilterInitialized()){
        return true;
    }
    state_est_.reset(new StateEstimator(new RBISResetUpdate(init_state,
                                                               init_cov,
                                                               RBISUpdateInterface::reset,
                                                               init_state.utime),
                                           history_span_));

    filter_initialized_ = true;
    if(verbose_){
        RCLCPP_INFO_STREAM(LOGGER, "Filter initialized.");
    }
    return true;
}

bool ROSFrontEnd::areModulesInitialized(){
    std::map<SensorId,bool>::iterator it = initialized_list_.begin();
    for(; it != initialized_list_.end(); ++it){
        if(!it->second){
            return false;
        }
    }
    return true;
}

bool ROSFrontEnd::isFilterInitialized(){
    return filter_initialized_;
}

}
