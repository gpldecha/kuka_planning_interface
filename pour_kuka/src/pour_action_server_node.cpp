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

   /* asrv::fexecuteCB learned_model_function = std::bind(&Pour_action_server::executeCB,
                                                        &pour_action_server,
                                                        std::placeholders::_1,
                                                        std::placeholders::_2,
                                                        std::placeholders::_3);*/

    asrv::fexecuteCB home_function = std::bind(&Pour_action_server::executeCB,
                                               &pour_action_server,
                                               std::placeholders::_1,
                                               std::placeholders::_2,
                                               std::placeholders::_3,
                                               Pour_action_server::PHASEHOME);

    asrv::fexecuteCB pour_function = std::bind(&Pour_action_server::executeCB,
                                               &pour_action_server,
                                               std::placeholders::_1,
                                               std::placeholders::_2,
                                               std::placeholders::_3,
                                               Pour_action_server::PHASEPOUR);

    asrv::fexecuteCB back_function = std::bind(&Pour_action_server::executeCB,
                                               &pour_action_server,
                                               std::placeholders::_1,
                                               std::placeholders::_2,
                                               std::placeholders::_3,
                                               Pour_action_server::PHASEBACK);


    action_server.push_back(home_function,"home");
    action_server.push_back(pour_function,"back");
    action_server.push_back(back_function,"pour");

    ros::spin();

    return 0;
}

