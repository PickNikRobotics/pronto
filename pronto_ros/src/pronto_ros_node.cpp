#include <rclcpp/rclcpp.hpp>
#include "pronto_ros/ros_frontend.hpp"
#include "pronto_ros/ins_ros_handler.hpp"
#include "pronto_ros/vicon_ros_handler.hpp"


using namespace pronto;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    std::shared_ptr<rclcpp::Node> node;
    node = std::make_shared<rclcpp::Node>("pronto_ros_node", node_options);
    ROSFrontEnd front_end(node);
    // get the list of active and init sensors from the param server
    typedef std::vector<std::string> SensorList;
    SensorList init_sensors;
    SensorList active_sensors;
    SensorList all_sensors;

    if(!node->get_parameter("init_sensors", init_sensors)){
        RCLCPP_ERROR(node->get_logger(), "Not able to get init_sensors param");
    }

    if(!node->get_parameter("active_sensors", active_sensors)){
        RCLCPP_ERROR(node->get_logger(), "Not able to get active_sensors param");
    }
    bool publish_pose = false;
    if(!node->get_parameter("publish_pose", publish_pose)){
        RCLCPP_WARN(node->get_logger(), "Not able to get publish_pose param. Not publishing pose.");
    }

    std::shared_ptr<InsHandlerROS> ins_handler_;
    std::shared_ptr<ViconHandlerROS> vicon_handler_;

    bool init = false;
    bool active = false;
    bool roll_forward = false;
    bool publish_head = false;
    std::string topic;


    for(SensorList::iterator it = active_sensors.begin(); it != active_sensors.end(); ++it){
        all_sensors.push_back(*it);
    }
    for(SensorList::iterator it = init_sensors.begin(); it != init_sensors.end(); ++it){
        all_sensors.push_back(*it);
    }

    for(SensorList::iterator it = all_sensors.begin(); it != all_sensors.end(); ++it)
    {
        RCLCPP_WARN_STREAM(node->get_logger(), "Getting params for sensor: " << *it);
        if(!node->get_parameter(*it + ".roll_forward_on_receive", roll_forward)){
            RCLCPP_WARN_STREAM(node->get_logger(), "Not adding sensor \"" << *it << "\".");
            RCLCPP_WARN_STREAM(node->get_logger(), "Param \"roll_forward_on_receive\" not available.");
            continue;
        }
        if(!node->get_parameter(*it + ".publish_head_on_message", publish_head)){
            RCLCPP_WARN_STREAM(node->get_logger(), "Not adding sensor \"" << *it << "\".");
            RCLCPP_WARN_STREAM(node->get_logger(), "Param \"publish_head_on_message\" not available.");
            continue;
        }
        if(!node->get_parameter(*it + ".topic", topic)){
            RCLCPP_WARN_STREAM(node->get_logger(), "Not adding sensor \"" << *it << "\".");
            RCLCPP_WARN_STREAM(node->get_logger(), "Param \"topic\" not available.");
            continue;
        }
        // check if the sensor is also used to initialize
        init = (std::find(init_sensors.begin(), init_sensors.end(), *it) != init_sensors.end());
        active = (std::find(active_sensors.begin(), active_sensors.end(), *it) != active_sensors.end());
        // is the IMU module in the list? Typically yes.
        if(it->compare("ins") == 0)
        {
            ins_handler_.reset(new InsHandlerROS(node));
            if(active){
                front_end.addSensingModule(*ins_handler_, *it, roll_forward, publish_head, topic);
            }
            if(init){
                front_end.addInitModule(*ins_handler_, *it, topic);
            }
        }
        if(it->compare("vicon") == 0 ){
            vicon_handler_.reset(new ViconHandlerROS(node));
            if(active){
                front_end.addSensingModule(*vicon_handler_, *it, roll_forward, publish_head, topic);
            }
            if(init){
                front_end.addInitModule(*vicon_handler_, *it, topic);
            }
        }
    }
    rclcpp::spin(node);
}
