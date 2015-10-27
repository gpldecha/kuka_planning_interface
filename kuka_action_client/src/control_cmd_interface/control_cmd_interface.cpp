#include "control_cmd_interface/control_cmd_interface.h"
#include <thread>
#include <boost/thread.hpp>

namespace ci{

Control_cmd_interface::Control_cmd_interface(ros::NodeHandle         &nh,
                                             kac::Kuka_action_client &kuka_action_client,
                                             const std::string       &service_name)
    :
    nh(nh),
    kuka_action_client(kuka_action_client),
    curr_action_state(action_states::PENDING)
{
    service             = nh.advertiseService(service_name,&Control_cmd_interface::service_callback,this);
    action_server_pub   = nh.advertise<std_msgs::String>("/kuka_server/cmd",10);
}

void Control_cmd_interface::init_nl_subscriber(std::string topic_name){
    subs.push_back(nh.subscribe(topic_name,1,&Control_cmd_interface::nl_command_callback,this));
}

void Control_cmd_interface::nl_command_callback(const std_msgs::String::ConstPtr& msg){

    std::string action_name = msg->data;
    ROS_INFO("I heard [%s]",action_name.c_str());

}

bool Control_cmd_interface::service_callback(kuka_action_client::String_cmd::Request& req,kuka_action_client::String_cmd::Response &res){

    std::cout<< "Control_cmd_interface::service_callback" << std::endl;

    std::string action_name         = req.cmd;
    std::string current_action_name = kuka_action_client.current_action_name;

    std::cout<< "=== Service call back === " <<                                 std::endl;
    std::cout<< " current action:        "   << current_action_name          << std::endl;
    std::cout<< " requested action:      "   << action_name                  << std::endl;

   if(!kuka_action_client.b_action_running){
    std::cout<< " start action:          "   << action_name << std::endl;
        worker_thread = boost::thread(&Control_cmd_interface::workThread,this,action_name);
    }else{
        kuka_action_client.ac_.cancelAllGoals();
        server_msg.data = "cancel";
        action_server_pub.publish(server_msg);
        ros::spinOnce();
        kuka_action_client.b_action_running = false;
        worker_thread.join();
    }
    res.res = "";
    return true;
}


void Control_cmd_interface::workThread(std::string action_name){
    kuka_action_client.call_action(action_name);
}



}
