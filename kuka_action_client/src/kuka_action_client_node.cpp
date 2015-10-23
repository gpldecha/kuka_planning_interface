#include "kuka_action_client/kuka_action_client.h"

#include <ros/ros.h>
#include "ros_param_parser/ros_param_parser.h"

int main(int argc,char** argv)
{

    std::map<std::string,std::string> param_name_value;
    param_name_value["speech_topic"]   = "/kuka_control/lib_cmd";
    param_name_value["service_topic"]  = "";

    ros::init(argc, argv, "control_cmd_interface");
    ros::NodeHandle nh("control_cmd_interface");

    if(!pps::parser_string(nh,param_name_value)){
        ROS_ERROR("failed to parse all parameters!");
        return -1;
    }



    return 0;
}
