#include "pronto_ros/lidar_odometry_ros_handler.hpp"
#include <pronto_ros/pronto_ros_conversions.hpp>

namespace pronto {

LidarOdometryHandlerROS::LidarOdometryHandlerROS(const rclcpp::Node::SharedPtr& node) : use_measurement_cov_(false), node_(node) {
  std::string prefix = "scan_matcher.";
  std::string mode_str = "pos";
  if(!node->get_parameter(prefix + "mode", mode_str)){
    RCLCPP_WARN_STREAM(node_->get_logger(), "Couldn't read param \"" << prefix << "mode\". Using position.");
  }
  if(mode_str.compare("position") == 0){
    cfg_.mode = LidarOdometryMode::POSITION;
    cfg_.cov_vo.resize(3,3);
    cfg_.cov_vo = Eigen::Matrix3d::Identity();
    cfg_.z_indices.resize(3);
    cfg_.z_indices = RBIS::positionInds();
  } else if(mode_str.compare("position_orient") == 0){
    cfg_.mode = LidarOdometryMode::POSITION_ORIENT;
    cfg_.cov_vo.resize(6,6);
    cfg_.cov_vo = Eigen::Matrix<double, 6,6>::Identity();
    cfg_.z_indices.resize(6);
    cfg_.z_indices.head<3>() = RBIS::positionInds();
    cfg_.z_indices.tail<3>() = RBIS::chiInds();
  } else if(mode_str.compare("position_yaw") == 0){
    cfg_.mode = LidarOdometryMode::POSITION_YAW;
    cfg_.cov_vo.resize(4,4);
    cfg_.cov_vo = Eigen::Matrix<double, 4,4>::Identity();
    cfg_.z_indices.resize(4);
    cfg_.z_indices.head<3>() = RBIS::positionInds();
    cfg_.z_indices.tail<1>() << RBIS::chi_ind + 2;
  } else {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Unsupported mode \"" << prefix << mode_str << "\". Using position.");
  }
  // before doing anything, set the covariance matrix to identity

  // unrealistic high covariances by default
  double r_pxy = 1;
  double r_pz = 1;
  double r_rxy = 50;
  double r_ryaw = 50;

  if(!node->get_parameter(prefix + "use_measurement_cov", use_measurement_cov_)){
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Coudn't read param \"" << prefix << "use_measurement_cov");
  }

  if(!node->get_parameter(prefix + "r_pxy", r_pxy)){
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Coudn't read param \"" << prefix << "r_px");
  }

  if(!node->get_parameter(prefix + "r_pz", r_pz)){
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Coudn't read param \"" << prefix << "r_pz");
  }
  utime_offset_ = 0;
  if(!node->get_parameter(prefix + "utime_offset", utime_offset_)){
    RCLCPP_WARN_STREAM(node_->get_logger(), "Couldnt' set time offset, set to 0.");
  }

  // square the standard deviation
  r_pxy = std::pow(r_pxy, 2);
  r_pz  = std::pow(r_pz, 2);

  cfg_.cov_vo.topLeftCorner<3,3>() = Eigen::Vector3d(r_pxy, r_pxy, r_pz).asDiagonal();

  if(cfg_.mode == LidarOdometryMode::POSITION_ORIENT || cfg_.mode == LidarOdometryMode::POSITION_YAW){
    if(!node->get_parameter(prefix + "r_rxy", r_rxy)){
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Coudn't read param \"" << prefix << "r_rxy");
    }
    if(!node->get_parameter(prefix + "r_ryaw", r_ryaw)){
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Coudn't read param \"" << prefix << "r_ryaw");
    }
    if(cfg_.mode == LidarOdometryMode::POSITION_ORIENT) {
      // by default the values are in degrees, convert into radians and square
      cfg_.cov_vo.bottomRightCorner<3,3>() = Eigen::Matrix3d::Identity() * std::pow((r_rxy * M_PI / 180.0), 2);
    }
    // the yaw element is common to both modes
    // note that this overwrites the last element if POSITION_ORIENT
    cfg_.cov_vo.bottomRightCorner<1,1>() << std::pow((r_ryaw * M_PI / 180.0), 2);
  }
  lidarodom_module_ = std::make_shared<LidarOdometryModule>(cfg_);
}

RBISUpdateInterface * LidarOdometryHandlerROS::processMessage(const pronto_msgs::msg::LidarOdometryUpdate *msg,
                                                               StateEstimator *state_estimator)
{

  pronto_msgs::msg::LidarOdometryUpdate mymsg = *msg;
  builtin_interfaces::msg::Duration msg_time_offset = rclcpp::Duration(utime_offset_ * 1e3);
  if( (msg->curr_timestamp.sec + msg->curr_timestamp.nanosec / 1e9)  < (msg->prev_timestamp.sec + msg->prev_timestamp.nanosec / 1e9)){
    RCLCPP_ERROR(node_->get_logger(), "[LidarOdometryHandlerROS] curr_timestamp is less than prev_timestamp! Ignoring message.");
    return nullptr;
  }

  try {
    mymsg.header.stamp.nanosec = msg->header.stamp.nanosec + msg_time_offset.nanosec;
    mymsg.header.stamp.sec = msg->header.stamp.sec + msg_time_offset.sec;
    mymsg.curr_timestamp.nanosec = msg->curr_timestamp.nanosec + msg_time_offset.nanosec;
    mymsg.curr_timestamp.sec = msg->curr_timestamp.sec + msg_time_offset.sec;
    mymsg.prev_timestamp.nanosec = msg->prev_timestamp.nanosec + msg_time_offset.nanosec;
    mymsg.prev_timestamp.sec = msg->prev_timestamp.sec + msg_time_offset.sec;
  } catch (std::runtime_error& ex) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "HEADER: " << msg->header.stamp.nanosec / 1e9 + msg->header.stamp.sec);
    RCLCPP_WARN_STREAM(node_->get_logger(), "CURR: " << msg->curr_timestamp.nanosec / 1e9 + msg->curr_timestamp.sec);
    RCLCPP_WARN_STREAM(node_->get_logger(), "PREV: " << msg->prev_timestamp.nanosec / 1e9 + msg->prev_timestamp.sec);
    RCLCPP_WARN_STREAM(node_->get_logger(), "OFFSET: " << msg_time_offset.nanosec / 1e9 + msg_time_offset.sec);
    RCLCPP_ERROR(node_->get_logger(), "Exception: [%s]", ex.what());
  }
  lidarOdometryFromROS(mymsg, lidarodom_update_);
  if(use_measurement_cov_ && cfg_.mode == LidarOdometryMode::POSITION){
    lidarodom_module_->setCovariance(lidarodom_update_.pose_covariance.topLeftCorner<3,3>());// position only
  }
  return lidarodom_module_->processMessage(&lidarodom_update_, state_estimator);
}

bool LidarOdometryHandlerROS::processMessageInit(const pronto_msgs::msg::LidarOdometryUpdate *msg,
                                                  const std::map<std::string, bool> &sensor_initialized,
                                                  const RBIS &default_state,
                                                  const RBIM &default_cov,
                                                  RBIS &init_state,
                                                  RBIM &init_cov)
{
  return true;
}

}
