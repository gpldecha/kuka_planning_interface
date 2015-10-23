#ifndef CONTROL_CMD_INTERFACE_H_
#define CONTROL_CMD_INTERFACE_H_

#include <ros/ros.h>

#include "control_cmd_interface/String_cmd.h"

#include <vector>
#include <functional>

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

namespace ci{


typedef std::function<bool(const std::string& action_name)> action_callback_func;


class Control_cmd_interface{

public:

    Control_cmd_interface(ros::NodeHandle& nh, const std::string &service_name);

    void init_nl_subscriber(std::string topic_name);

    void register_callback(action_callback_func& function);

private:

    void nl_command_callback(const std_msgs::String::ConstPtr &msg);

    bool service_callback(control_cmd_interface::String_cmd::Request& req,control_cmd_interface::String_cmd::Response &res);

private:

    ros::NodeHandle&                nh;
    std::vector<ros::Subscriber>    subs;
    action_callback_func*           ptr_action_callback;
    ros::ServiceServer              service;

};

}

#endif
