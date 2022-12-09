#include "pronto_ros/visual_odometry_ros_handler.hpp"
#include <pronto_ros/pronto_ros_conversions.hpp>

namespace pronto {

VisualOdometryHandlerROS::VisualOdometryHandlerROS(const rclcpp::Node::SharedPtr& node) : node_(node) {
  std::string prefix = "fovis.";
  VisualOdometryConfig cfg;
  std::string mode_str = "pos";
  if(!node_->get_parameter(prefix + "mode", mode_str)){
    RCLCPP_WARN_STREAM(node_->get_logger(), "Couldn't read param \"" << prefix << "mode\". Using position.");
  }
  if(mode_str.compare("pos") == 0){
    cfg.mode = VisualOdometryMode::MODE_POSITION;
    cfg.cov_vo.resize(3,3);
    cfg.cov_vo = Eigen::Matrix3d::Identity();
    cfg.z_indices.resize(3);
    cfg.z_indices = RBIS::positionInds();
  } else if(mode_str.compare("pos_orient") == 0){
    cfg.mode = VisualOdometryMode::MODE_POSITION_ORIENT;
    cfg.cov_vo.resize(6,6);
    cfg.cov_vo = Eigen::Matrix<double, 6,6>::Identity();
    cfg.z_indices.resize(6);
    cfg.z_indices.head<3>() = RBIS::positionInds();
    cfg.z_indices.tail<3>() = RBIS::chiInds();
  } else {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Unsupported mode \"" << prefix << mode_str << "\". Using position.");
  }
  // before doing anything, set the covariance matrix to identity


  // unrealistic high covariances by default
  double r_px = 1;
  double r_py = 1;
  double r_pz = 1;
  double r_rxy = 50;
  double r_ryaw = 50;

  if(!node_->get_parameter(prefix + "r_px", r_px)){
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Coudn't read param \"" << prefix << "r_px");
  }
  if(!node_->get_parameter(prefix + "r_py", r_py)){
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Coudn't read param \"" << prefix << "r_py");
  }
  if(!node_->get_parameter(prefix + "r_pz", r_pz)){
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Coudn't read param \"" << prefix << "r_pz");
  }
  utime_offset_ = 0;
  if(!node_->get_parameter(prefix + "utime_offset", utime_offset_)){
    RCLCPP_WARN_STREAM(node_->get_logger(), "Couldnt' set time offset, set to 0.");
  }

  // square the standard deviation
  r_px = std::pow(r_px, 2);
  r_py = std::pow(r_py, 2);
  r_pz  = std::pow(r_pz, 2);

  cfg.cov_vo.topLeftCorner<3,3>() = Eigen::Vector3d(r_px, r_py, r_pz).asDiagonal();

  if(cfg.mode == VisualOdometryMode::MODE_POSITION_ORIENT){
    if(!node_->get_parameter(prefix + "r_rxy", r_rxy)){
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Coudn't read param \"" << prefix << "r_rxy");
    }
    if(!node_->get_parameter(prefix + "r_ryaw", r_ryaw)){
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Coudn't read param \"" << prefix << "r_ryaw");
    }
    // by default the values are in degrees, convert into radians and square
    cfg.cov_vo.bottomRightCorner<3,3>() = Eigen::Matrix3d::Identity() * std::pow((r_rxy * M_PI / 180.0), 2);
    // the yaw element is different than the rest
    cfg.cov_vo.bottomRightCorner<1,1>() << std::pow((r_ryaw * M_PI / 180.0), 2);
  }
  vo_module_ = std::make_shared<VisualOdometryModule>(cfg);
}

RBISUpdateInterface * VisualOdometryHandlerROS::processMessage(const pronto_msgs::msg::VisualOdometryUpdate *msg,
                                                               StateEstimator *state_estimator)
{

  pronto_msgs::msg::VisualOdometryUpdate mymsg = *msg;
  builtin_interfaces::msg::Duration msg_time_offset = rclcpp::Duration::from_seconds(utime_offset_ * 1e6);
  try {
    mymsg.header.stamp.nanosec = msg->header.stamp.nanosec + msg_time_offset.nanosec;
    mymsg.header.stamp.sec = msg->header.stamp.sec + msg_time_offset.sec;
    mymsg.curr_timestamp.nanosec = msg->curr_timestamp.nanosec + msg_time_offset.nanosec;
    mymsg.curr_timestamp.sec = msg->curr_timestamp.sec + msg_time_offset.sec;
    mymsg.prev_timestamp.nanosec = msg->prev_timestamp.nanosec + msg_time_offset.nanosec;
    mymsg.prev_timestamp.sec = msg->prev_timestamp.sec + msg_time_offset.sec;
  } catch (std::runtime_error& ex) {
    RCLCPP_WARN_STREAM(node_->get_logger(), "HEADER: " << msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec);
    RCLCPP_WARN_STREAM(node_->get_logger(), "CURR: " << msg->curr_timestamp.sec * 1e9 +  msg->curr_timestamp.nanosec);
    RCLCPP_WARN_STREAM(node_->get_logger(), "PREV: " << msg->prev_timestamp.sec * 1e9 +  msg->prev_timestamp.nanosec);
    RCLCPP_WARN_STREAM(node_->get_logger(), "OFFSET: " << msg_time_offset.sec * 1e9 + msg_time_offset.nanosec);
    RCLCPP_ERROR(node_->get_logger(), "Exception: [%s]", ex.what());
  }
  visualOdometryFromROS(mymsg, vo_update_);
  return vo_module_->processMessage(&vo_update_, state_estimator);
}

bool VisualOdometryHandlerROS::processMessageInit(const pronto_msgs::msg::VisualOdometryUpdate *msg,
                                                  const std::map<std::string, bool> &sensor_initialized,
                                                  const RBIS &default_state,
                                                  const RBIM &default_cov,
                                                  RBIS &init_state,
                                                  RBIM &init_cov)
{
  return true;
}

}  // namespace pronto
