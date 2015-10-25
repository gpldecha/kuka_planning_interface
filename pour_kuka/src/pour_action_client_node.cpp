#include <ros/ros.h>

#include "control_cmd_interface/control_cmd_interface.h"
#include "pour_kuka/pour_action_client.h"
#include "kuka_action_client/kuka_action_client.h"
#include "ros_param_parser/ros_param_parser.h"

int main(int argc,char** argv){

    ros::init(argc, argv,"control_cmd_interface");
    ros::NodeHandle nh("control_cmd_interface");

    std::string node_name = ros::this_node::getName();

    std::map<std::string,std::string> param_name_value;
    param_name_value[node_name + "/speech_topic"]           = "";
    param_name_value[node_name + "/service_name"]           = "";
    param_name_value[node_name + "/action_server_name"]     = "";

    if(!pps::parser_string(nh,param_name_value)){
        ROS_ERROR("failed to parse all parameters!");
        return -1;
    }
    std::string speech_topic       =  param_name_value[node_name + "/speech_topic"];
    std::string serivce_name       = param_name_value[node_name + "/service_name"];
    std::string action_server_name =  param_name_value[node_name + "/action_server_name"];


    /** ------------- Initialise Action Client & Set Action-Goals -------------

      The Pour_client is initialsed. A set of actions and goals are defined
      add added to the action clients container which is a map. The key of
      the map is the name of the action and the value is the Goal.

    **/
    Pour_client pour_client(action_server_name);


    /**  ------------- Initialise Control cmd  interface  -------------
     *  The control command interface is an interface to the action client.
     *  It provied a ros service and a voice command interface such to
     *  command the client server to send desired action requests to the action server.
     */
    ci::Control_cmd_interface control_cmd_interface(nh,pour_client,serivce_name);
    control_cmd_interface.init_nl_subscriber(speech_topic);


    ros::spin();

    return 0;
}
