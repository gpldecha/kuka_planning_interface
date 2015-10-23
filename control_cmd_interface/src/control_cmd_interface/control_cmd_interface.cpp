#include "control_cmd_interface/control_cmd_interface.h"

namespace ci{

Control_cmd_interface::Control_cmd_interface(ros::NodeHandle& nh,const std::string& service_name)
    :nh(nh)
{
    ptr_action_callback = NULL;
    service             = nh.advertiseService(service_name,&Control_cmd_interface::service_callback,this);

}

void Control_cmd_interface::register_callback(action_callback_func& callback){
    ptr_action_callback = &callback;
}

void Control_cmd_interface::init_nl_subscriber(std::string topic_name){
    subs.push_back(nh.subscribe(topic_name,1,&Control_cmd_interface::nl_command_callback,this));
}

void Control_cmd_interface::nl_command_callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("I heard [%s]",msg->data.c_str());

    if(ptr_action_callback!=NULL){
        (*ptr_action_callback)(msg->data);
    }else{
        ROS_WARN("Control_cmd_interface::ptr_action_callback is NULL");
    }
}

bool Control_cmd_interface::service_callback(control_cmd_interface::String_cmd::Request& req,control_cmd_interface::String_cmd::Response &res){

    std::string action_name = req.str;
    ROS_INFO("serice callback: [%s]",action_name.c_str());
    if(ptr_action_callback!=NULL){
        (*ptr_action_callback)(action_name);
    }else{
        ROS_WARN("Control_cmd_interface::ptr_action_callback is NULL");
    }

    return true;
}



}
