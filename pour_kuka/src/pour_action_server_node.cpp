#include "pour_kuka/pour_action_server.h"
#include <functional>
#include "ros_param_parser/ros_param_parser.h"
#include "kuka_common_action_server/kuka_goto_cart_as.h"



int main(int argc, char** argv) {

    // ----------- Launch ros node ----------------------

    ros::init(argc, argv, "plan2ctrl");
    ROS_INFO("Initializing Server");
    ros::NodeHandle nh;

    // ----------- Get parameters (parameter sever) ------

    std::string node_name = ros::this_node::getName();
    std::map<std::string,std::string> param_name_value;
    param_name_value[node_name + "/action_server_name"]  = "";

    if(!pps::parser_string(nh,param_name_value)){
        ROS_ERROR("failed to parse all parameters!");
        return -1;
    }
    pps::parser_print(param_name_value);
    std::string action_server_name  =  param_name_value[node_name + "/action_server_name"];

    /**  ------------- Initialise various control policies -------------
     *
     *  The user initalises his control polices here. For example load all the GMM parameters
     *  etc.. All policies implement the base class action server. Once all the polices
     *  are initialised they can be added to the action server. In this example the Pour
     *  control policy is initialised.
     *
     **/

    // Pour action set
    Pour_action_server pour_action_server(nh);
    pour_action_server.initialize();

    // Goto actions
    asrv::Action_ee_initialiser action_ee_init;
    action_ee_init.action_name = "goto_home";
    asrv::Kuka_goto_cart_as kuka_goto_cart_as(nh,action_ee_init);


    /**  ------------- Initialise Action Server -------------
     *
     *    The action server takes care of handling the execution of all the possible control policies
     *    that the user has defined on the simulated or physical robot. The action client sends
     *    action requests and interuptions to the action server.
     *    All the user has to do add all his policy implemenations to the action server.
     *
     **/

    asrv::Action_server action_server(nh,action_server_name);


    /**
        Here there different actions (home,back and pour) are added to the action server.
        You will not that the pointer is the same for each push_back call. This is because
        The Pour_action_server implements three actions. But you could have one action per
        class implementation.

    **/

    action_server.push_back(&pour_action_server,"home");
    action_server.push_back(&pour_action_server,"back");
    action_server.push_back(&pour_action_server,"pour");
    action_server.push_back(&kuka_goto_cart_as,"goto_home");

    ros::spin();

    return 0;
}

