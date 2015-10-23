#include "control_cmd_interface/control_cmd_interface.h"

#include <ros/ros.h>
#include "ros_param_parser/ros_param_parser.h"

#include "std_msgs/String.h"



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

    // --------------  Control cmd interface ----------------
    ci::Control_cmd_interface control_cmd_interface(nh,"kuka_cmd");
    control_cmd_interface.init_nl_subscriber(param_name_value["speech_topic"]);

    ros::spin();


    return 0;
}
