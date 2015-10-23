#include "pour_kuka/pour_action_server.h"
#include <functional>
#include "ros_param_parser/ros_param_parser.h"



int main(int argc, char** argv) {

    ros::init(argc, argv, "plan2ctrl");
    ROS_INFO("Initializing Server");
    ros::NodeHandle nh;


    std::string node_name = ros::this_node::getName();

    std::map<std::string,std::string> param_name_value;
    param_name_value[node_name + "/action_server_name"]           = "";

    if(!pps::parser_string(nh,param_name_value)){
        ROS_ERROR("failed to parse all parameters!");
        return -1;
    }

    pps::parser_print(param_name_value);

    std::string action_server_name  =  param_name_value[node_name + "/action_server_name"];


    asrv::Action_server action_server(nh,action_server_name);

    Pour_action_server pour_action_server(nh);
    pour_action_server.initialize();

    asrv::fexecuteCB learned_model_function = std::bind(&Pour_action_server::executeCB,
                                                        &pour_action_server,
                                                        std::placeholders::_1,
                                                        std::placeholders::_2,
                                                        std::placeholders::_3);

    pour_action_server.push_back(learned_model_function,"LEARNED_MODEL");

    ros::spin();

    return 0;
}

