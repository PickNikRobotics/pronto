#pragma once

#include <pronto_core/sensing_module.hpp>
#include <pronto_core/state_est.hpp>
#include <pronto_core/rotations.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <chrono>
#include <tf2_ros/transform_broadcaster.h>
#include <string>
#include <cstdlib>
#include <cxxabi.h>
#include <nav_msgs/msg/path.hpp>
#include <any>


static const rclcpp::Logger LOGGER = rclcpp::get_logger("ros_frontend");

template<typename T>
std::string type_name()
{
    int status;
    std::string tname = typeid(T).name();
    char *demangled_name = abi::__cxa_demangle(tname.c_str(), NULL, NULL, &status);
    if(status == 0) {
        tname = demangled_name;
        std::free(demangled_name);
    }
    return tname;
}

namespace pronto {
class ROSFrontEnd {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    using SensorId = std::string;

    ROSFrontEnd(const std::shared_ptr<rclcpp::Node> &node, bool verbose = false);
    virtual ~ROSFrontEnd();

    template<class MsgT>
    void addSensingModule(SensingModule<MsgT>& module,
                          const SensorId& sensor_id,
                          bool roll_forward,
                          bool publish_head,
                          const std::string& topic,
                          bool subscribe = true);

    template<class MsgT, class SecondaryMsgT>
    inline void addSecondarySensingModule(DualSensingModule<MsgT, SecondaryMsgT>& /*module*/,
                                          const SensorId& sensor_id,
                                          const std::string& topic,
                                          bool subscribe)
    {
        if(!subscribe){
            return;
        }
        RCLCPP_INFO_STREAM(LOGGER, sensor_id << " subscribing to " << topic
                        << " with SecondaryMsgT = " << type_name<SecondaryMsgT>());

        std::function<void(std::shared_ptr<const SecondaryMsgT>)> f = std::bind(&ROSFrontEnd::secondaryCallback<MsgT, SecondaryMsgT>, this, std::placeholders::_1, sensor_id);
        secondary_subscribers_.push_back(node_->create_subscription<SecondaryMsgT>(topic, 10, f));
    }


    template <class MsgT>
    void addInitModule(SensingModule<MsgT>& module,
                       const SensorId& sensor_id,
                       const std::string& topic,
                       bool subscribe = true);

    bool areModulesInitialized();

    bool isFilterInitialized();

    inline void getState(RBIS& state, RBIM& cov) const{
        state_est_->getHeadState(state, cov);
    }

    inline bool reset(const RBIS& state, const RBIM& cov) {
        state_est_->addUpdate(new pronto::RBISResetUpdate(state,
                                                               cov,
                                                               RBISUpdateInterface::reset,
                                                               state.utime), true);
        return true;
    }
    template <class MsgT>
    void initCallback(std::shared_ptr<MsgT const> msg,
                      const SensorId& Key);

    template <class PrimaryMsgT, class SecondaryMsgT>
    void secondaryCallback(std::shared_ptr<SecondaryMsgT const> msg,
                           const SensorId& sensor_id);

    template <class MsgT>
    void callback(std::shared_ptr<MsgT const> msg,
                  const SensorId& Key);
protected:
    bool initializeFilter();

    void initializeState();
    void initializeCovariance();

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<StateEstimator> state_est_;
    std::map<SensorId, void*> active_modules_;
    std::map<SensorId, void*> init_modules_;
    std::map<SensorId, bool> initialized_list_;
    std::map<SensorId, bool> roll_forward_;
    std::map<SensorId, bool> publish_head_;

    RBIS default_state;
    RBIM default_cov;

    RBIS init_state;
    RBIM init_cov;

    RBIS head_state;
    RBIM head_cov;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_pub_;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> init_subscribers_;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> sensors_subscribers_;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> secondary_subscribers_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster_;
    geometry_msgs::msg::TransformStamped transform_msg_;
    bool publish_tf_ = false;

    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg_;
    geometry_msgs::msg::TwistWithCovarianceStamped twist_msg_;

    nav_msgs::msg::Path aicp_path;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr aicp_path_publisher_;

    uint64_t history_span_;

    tf2::Vector3 temp_v3;
    tf2::Quaternion temp_q;

    bool filter_initialized_ = false;
    bool verbose_ = true;
};
}  // namespace pronto

namespace pronto {
template <class MsgT>
void ROSFrontEnd::addInitModule(SensingModule<MsgT>& module,
                                const SensorId& sensor_id,
                                const std::string& topic,
                                bool subscribe)
{
    if(init_modules_.count(sensor_id) > 0){
        RCLCPP_WARN_STREAM(LOGGER, "Init Module \"" << sensor_id << "\" already added. Skipping.");
        return;
    }
    RCLCPP_INFO_STREAM(LOGGER, "Initializing module: " << sensor_id);
    RCLCPP_INFO_STREAM(LOGGER, "Sensor init id: " << sensor_id);
    RCLCPP_INFO_STREAM(LOGGER, "Topic: " << topic);

    // add the sensor to the list of sensor that require initialization
    std::pair<SensorId, bool> init_id_pair(sensor_id, false);
    initialized_list_.insert(init_id_pair);
    // store the module as void*, to allow for different types of module to stay
    // in the same container. The type will be known when the message arrives
    // so we can properly cast back to the right type.
    std::pair<SensorId, void*> pair(sensor_id, (void*) &module);
    init_modules_.insert(pair);
    if(subscribe){
        std::cerr << sensor_id << " subscribing to " << topic;
        std::cerr << " with MsgT = " << type_name<MsgT>() << std::endl;
        std::cerr << "Initializing subscriber for: " << topic << std::endl;
        std::function<void(std::shared_ptr<const MsgT>)> f = std::bind(&ROSFrontEnd::initCallback<MsgT>, this, std::placeholders::_1, sensor_id);
        std::cerr << "Initialized function subscriber for: " << topic << std::endl;
        init_subscribers_.push_back(node_->create_subscription<MsgT>(topic,
                            10,
                            f));
        std::cerr << "Done initializing subscriber for: " << topic << std::endl;
    }
}

template<class MsgT>
void ROSFrontEnd::addSensingModule(SensingModule<MsgT>& module,
                                   const SensorId& sensor_id,
                                   bool roll_forward,
                                   bool publish_head,
                                   const std::string& topic,
                                   bool subscribe)
{
    // int this implementation we allow only one different type of module
    if(active_modules_.count(sensor_id) > 0){
        RCLCPP_WARN_STREAM(LOGGER, "Sensing Module \"" << sensor_id << "\" already added. Skipping.");
        return;
    }

    RCLCPP_INFO_STREAM(LOGGER, "Activating module: " << sensor_id);
    RCLCPP_INFO_STREAM(LOGGER, "Sensor id: " << sensor_id);
    RCLCPP_INFO_STREAM(LOGGER, "Roll forward: "<< (roll_forward? "yes" : "no"));
    RCLCPP_INFO_STREAM(LOGGER, "Publish head: "<< (publish_head? "yes" : "no"));
    RCLCPP_INFO_STREAM(LOGGER, "Topic: " << topic);

    // store the will to roll forward when the message is received
    std::pair<SensorId, bool> roll_pair(sensor_id, roll_forward);
    roll_forward_.insert(roll_pair);

    // store the will to publish the estimator state when the message is received
    std::pair<SensorId, bool> publish_pair(sensor_id, publish_head);
    publish_head_.insert(publish_pair);

    // store the module as void*, to allow for different types of module to stay
    // in the same container. The type will be known when the message arrives
    // so we can properly cast back to the right type.
    std::pair<SensorId, void*> pair(sensor_id, (SensingModule<MsgT>*) &module);
    active_modules_.insert(pair);
    // subscribe the generic templated callback for all modules
    if(subscribe){
        std::cerr << sensor_id << " subscribing to " << topic;
        std::cerr << " with MsgT = " << type_name<MsgT>() << std::endl;
        std::cerr << "Activating subscriber for: " << topic << std::endl;
        std::function<void(std::shared_ptr<const MsgT>)> f = std::bind(&ROSFrontEnd::callback<MsgT>, this, std::placeholders::_1, sensor_id);
        std::cerr << "Activated function subcriber for: " << topic << std::endl;;
        sensors_subscribers_.push_back(node_->create_subscription<MsgT>(topic,
                                            10,
                                            f));
        std::cerr << "Done activating subscriber for: " << topic << std::endl;
    }
}



template <class MsgT>
void ROSFrontEnd::initCallback(std::shared_ptr<MsgT const> msg, const SensorId& sensor_id){
    if(verbose_){
        RCLCPP_INFO_STREAM(LOGGER, "Init callback for sensor " << sensor_id);
    }
    if(initialized_list_.count(sensor_id) > 0 && !initialized_list_[sensor_id])
    {
        initialized_list_[sensor_id] = static_cast<SensingModule<MsgT>*>(init_modules_[sensor_id])->processMessageInit(msg.get(),
                                                                                                                       initialized_list_,
                                                                                                                       default_state,
                                                                                                                       default_cov,
                                                                                                                       init_state,
                                                                                                                       init_cov);

        // if the sensor has been successfully initialized, we unsubscribe.
        // This happens only for the sensors which are only for initialization.
        // The sensor which are for initialization and active will continue to listen
        if(initialized_list_[sensor_id]){
            // attempt to initialize the filter, because the value has changed
            // in the list
            initializeFilter();
        }
    } else {
        // if we are here it means that the module is not in the list of
        // initialized modules or that the module is already initialized
        // in both cases we don't want to subscribe to this topic anymore,
        // unless there is no subscriber because we are processing a rosbag.
        // if(init_subscribers_.count(sensor_id) > 0){
        // }
    }
}

//TODO come up with a better way to activate / deactivate debug mode
#define DEBUG_MODE 0

template <class MsgT>
void ROSFrontEnd::callback(std::shared_ptr<MsgT const> msg, const SensorId& sensor_id)
{
#if DEBUG_MODE
    RCLCPP_INFO_STREAM(LOGGER, "Callback for sensor " << sensor_id);
#endif
    // this is a generic templated callback that does the same for every module:
    // if the module is initialized and the filter is ready
    // 1) take the measurement update and pass it to the filter if valid
    // 2) publish the filter state if the module wants to
    if(isFilterInitialized()) {
        // appropriate casting to the right type and call to the process message
        // function to get the update
        // Record start time
#if DEBUG_MODE
        auto start = std::chrono::high_resolution_clock::now();
#endif
        RBISUpdateInterface* update = static_cast<SensingModule<MsgT>*>(active_modules_[sensor_id])->processMessage(msg.get(), state_est_.get());
#if DEBUG_MODE
        auto end = std::chrono::high_resolution_clock::now();
        RCLCPP_INFO_STREAM(LOGGER, "Time elapsed process message: " << std::chrono::duration_cast<std::chrono::microseconds>(end -start).count());
        start = end;
#endif
        // if the update is invalid, we leave
        if(update == nullptr){
#if DEBUG_MODE
            RCLCPP_INFO_STREAM(LOGGER, "Invalid " << sensor_id << " update" << std::endl);
#endif
            // special case for pose meas, it returns null when it does not want
            // to process data anymore
            // if(sensor_id.compare("pose_meas") == 0){
            //     sensors_subscribers_["pose_meas"].shutdown();
            // }
            return;
        }
#if DEBUG_MODE
        if(sensor_id.compare("fovis") == 0){
          RCLCPP_INFO_STREAM(LOGGER, "fovis update posterior: " << update->posterior_state.position().transpose());
        }
#endif
#define DEBUG_AICP 0
#if DEBUG_AICP

        if(sensor_id.compare("scan_matcher") == 0){
          aicp_path.header.frame_id = "odom";
          Eigen::Vector3d p = dynamic_cast<RBISIndexedPlusOrientationMeasurement*>(update)->measurement.head<3>();
          Eigen::Quaterniond q = dynamic_cast<RBISIndexedPlusOrientationMeasurement*>(update)->orientation;

          std::cerr << "MEASR    : " << p.transpose() << "   " << rotation::getEulerAnglesDeg(q).transpose() << std::endl;

         // Eigen::Vector3d p = dynamic_cast<RBISIndexedMeasurement*>(update)->measurement.head<3>();

          // std::cerr << "ABOUT TO SEND TO FILTER THE FOLLOWING: " << p.transpose() << std::endl;

        }
#endif
        RBIS prior;
        RBIS posterior;
        RBIM prior_cov;
        RBIM posterior_cov;
        state_est_->getHeadState(prior,prior_cov);
        // tell also the filter if we need to roll forward
        state_est_->addUpdate(update, roll_forward_[sensor_id]);
        state_est_->getHeadState(posterior,posterior_cov);

if(sensor_id.compare("scan_matcher") == 0){
    std::cerr << "PRIOR    : " << prior.position().transpose() << " " << rotation::getEulerAnglesDeg(prior.orientation()).transpose() << std::endl;
    std::cerr << "POSTERIOR: " << posterior.position().transpose() << " " << rotation::getEulerAnglesDeg(posterior.orientation()).transpose() << std::endl;
    std::cerr << ":::::::" << std::endl;
}

#if DEBUG_MODE
        end = std::chrono::high_resolution_clock::now();
        RCLCPP_INFO_STREAM(LOGGER, "Time elapsed process addUpdate: " << std::chrono::duration_cast<std::chrono::microseconds>(end -start).count());
        start = end;
#endif
        if(publish_head_[sensor_id]){
            state_est_->getHeadState(head_state, head_cov);

            // fill in linear velocity
            twist_msg_.twist.twist.linear = tf2::toMsg2(head_state.velocity());
            // fill in angular velocity
            twist_msg_.twist.twist.angular = tf2::toMsg2(head_state.angularVelocity());

            // fill in time
            twist_msg_.header.stamp = rclcpp::Time(head_state.utime * 1000);

            // TODO insert appropriate covariance into the message

            // set twist covariance to zero
            // std::fill(twist_msg_.twist.covariance, twist_msg_.twist.covariance+twist_msg_.twist.covariance.size(), 0)
            Eigen::Block<RBIM, 3, 3> vel_cov = head_cov.block<3,3>(RBIS::velocity_ind, RBIS::velocity_ind);
            Eigen::Block<RBIM, 3, 3> omega_cov = head_cov.block<3,3>(RBIS::angular_velocity_ind, RBIS::angular_velocity_ind);

            for(int i=0; i<3; i++){
              for(int j=0; j<3; j++){
                twist_msg_.twist.covariance[6*i+j] = vel_cov(i,j);
                twist_msg_.twist.covariance[6*(i+3)+j+3] = omega_cov(i,j);
              }
            }


            // publish the twist
            twist_pub_->publish(twist_msg_);

            // make sure stuff is non-NAN before publishing
            assert(head_state.position().allFinite());

            // fill in message position
            geometry_msgs::msg::Vector3 temp_gv3 = tf2::toMsg2(head_state.position());
            pose_msg_.pose.pose.position.x = temp_gv3.x;
            pose_msg_.pose.pose.position.y = temp_gv3.y;
            pose_msg_.pose.pose.position.z = temp_gv3.z;

            // fill in message orientation
            pose_msg_.pose.pose.orientation = tf2::toMsg(head_state.orientation());

            // fill in time
            pose_msg_.header.stamp = rclcpp::Time(head_state.utime * 1000);
            if(publish_tf_){
                // Only publish the pose if the timestamp is different:
                // This prevents issues in Noetic where repeated warnings of type:
                // "TF_REPEATED_DATA ignoring data with redundant timestamp for frame base at time"
                // are otherwise printed to the terminal.
                // Cf. https://github.com/ros/geometry2/issues/467#issuecomment-751572836
                rclcpp::Time new_stamp = pose_msg_.header.stamp;
                if (new_stamp > transform_msg_.header.stamp) {
                    transform_msg_.transform.translation.x = pose_msg_.pose.pose.position.x;
                    transform_msg_.transform.translation.y = pose_msg_.pose.pose.position.y;
                    transform_msg_.transform.translation.z = pose_msg_.pose.pose.position.z;
                    transform_msg_.transform.rotation = pose_msg_.pose.pose.orientation;
                    transform_msg_.header.stamp = new_stamp;
                    tf2_broadcaster_->sendTransform(transform_msg_);
                }
            }

            // TODO insert appropriate covariance into the message
            // publish the pose
            pose_pub_->publish(pose_msg_);
        }
#if DEBUG_MODE
        else {
            RCLCPP_WARN(LOGGER, "NOT Publish head sensor ID");
        }
        end = std::chrono::high_resolution_clock::now();

        RCLCPP_INFO_STREAM(LOGGER, "Time elapsed till the end: " << std::chrono::duration_cast<std::chrono::microseconds>(end -start).count());
        std::cout << std::endl;
#endif
    }
}

template <class PrimaryMsgT, class SecondaryMsgT>
void ROSFrontEnd::secondaryCallback(std::shared_ptr<SecondaryMsgT const> msg,
                                    const SensorId& sensor_id)
{    
    auto a = dynamic_cast<DualSensingModule<PrimaryMsgT,SecondaryMsgT>*>(static_cast<SensingModule<PrimaryMsgT>*>(active_modules_[sensor_id]));
    a->processSecondaryMessage(*msg);
}

} // namespace pronto
