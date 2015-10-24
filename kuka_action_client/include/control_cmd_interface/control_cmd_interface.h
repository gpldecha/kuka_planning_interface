#ifndef CONTROL_CMD_INTERFACE_H_
#define CONTROL_CMD_INTERFACE_H_

#include <ros/ros.h>

#include "control_cmd_interface/String_cmd.h"
#include "kuka_action_client/kuka_action_client.h"

#include <vector>
#include <functional>
#include <future>

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

namespace ci{


typedef actionlib::SimpleClientGoalState::StateEnum action_states;

class Control_cmd_interface{


public:

    Control_cmd_interface(ros::NodeHandle          &nh,
                          kac::Kuka_action_client  &kuka_action_client,
                          const std::string        &service_name);

    void init_nl_subscriber(std::string topic_name);

private:

    void nl_command_callback(const std_msgs::String::ConstPtr &msg);

    bool service_callback(control_cmd_interface::String_cmd::Request& req,control_cmd_interface::String_cmd::Response &res);

    void workThread(std::string action_name);

private:

    ros::NodeHandle&                 nh;
    kac::Kuka_action_client&         kuka_action_client;
    std::vector<ros::Subscriber>     subs;
    boost::thread                    worker_thread;
    ros::ServiceServer               service;
    ros::Publisher                   action_server_pub;


    actionlib::SimpleClientGoalState curr_action_state;
    std_msgs::String                 server_msg;
};

}

#endif
