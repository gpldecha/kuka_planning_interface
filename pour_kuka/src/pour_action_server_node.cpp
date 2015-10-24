#include "pour_kuka/pour_action_server.h"
#include <functional>
#include "ros_param_parser/ros_param_parser.h"



int main(int argc, char** argv) {

    ros::init(argc, argv, "plan2ctrl");
    ROS_INFO("Initializing Server");
    ros::NodeHandle nh;

    std::string node_name = ros::this_node::getName();

    std::map<std::string,std::string> param_name_value;
    param_name_value[node_name + "/action_server_name"]  = "";

    if(!pps::parser_string(nh,param_name_value)){
        ROS_ERROR("failed to parse all parameters!");
        return -1;
    }

    pps::parser_print(param_name_value);

    std::string action_server_name  =  param_name_value[node_name + "/action_server_name"];

    asrv::Action_server action_server(nh,action_server_name);

    Pour_action_server pour_action_server(nh);
    pour_action_server.initialize();

    action_server.push_back(&pour_action_server,"home");
    action_server.push_back(&pour_action_server,"back");
    action_server.push_back(&pour_action_server,"pour");

    ros::spin();

    return 0;
}

