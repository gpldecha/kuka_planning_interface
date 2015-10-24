#include "kuka_action_server/action_server.h"

namespace asrv {

Action_server::Action_server(ros::NodeHandle& nh,std::string name):
    as_(nh, name, boost::bind(&Action_server::executeCB, this, _1), false),
    action_name_(name)
{
    as_.start();

    as_sub             = nh.subscribe("/kuka_server/cmd", 10, &Action_server::subscriber_cb,this);
    base_action_server = NULL;

}

void Action_server::push_back(fexecuteCB& function,std::string action_name){
    actions[action_name] = &function;
}

void Action_server::push_back(Base_action_server* base_action_server,std::string action_name){
    actions2[action_name] = base_action_server;
}

void Action_server::executeCB(const cptrGoal& goal){

   // std::string desired_action = goal->action_type;
    //ROS_INFO_STREAM( "Desired Action is " << desired_action);

   /* (*ptr_isOkay) = false;
    ros::Rate r(10);
    ROS_INFO("Waiting for EE pose/ft topic...");
    while(ros::ok() && (!(*ptr_isOkay))) {
        r.sleep();
    }*/

    if(!ros::ok()) {
        result_.success = 0;
        ROS_INFO("%s: Failed", action_name_.c_str());
        as_.setAborted(result_);
        return;
    }


    // initialize action progress as null
    feedback_.progress = 0;

    ///////////////////////////////////////////////
    /////----- EXECUTE REQUESTED ACTION ------/////
    ///////////////////////////////////////////////

    std::cout<< "EXECUTE REQUESTED ACTION" << std::endl;

    std::string action_type = goal->action_type;
    std::string action_name = goal->action_name;
    std::cout<< "action_type: " << action_type << std::endl;
    std::cout<< "action_name: " << action_name << std::endl;

   // actions_it              = actions.find(action_name);
    actions2_it              = actions2.find(action_name);


    if(actions2_it == actions2.end()){
        ROS_ERROR_STREAM("Unidentified action name "<< action_name.c_str());
        result_.success = false;
        as_.setAborted(result_);
    }else{

        base_action_server = actions2_it->second;
        base_action_server->bBaseRun = true;
        bool success                 = base_action_server->execute_CB(as_,feedback_,goal);


        result_.success = success;
        if(success)
        {
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            as_.setSucceeded(result_);
        } else {
            ROS_INFO("%s: Failed", action_name_.c_str());
            as_.setAborted(result_);
        }
    }
}


void Action_server::subscriber_cb(const std_msgs::String::ConstPtr& msg){

    std::string cmd = msg->data.c_str();
    ROS_INFO("ACTION SERVER SUBSCRIBER cmd [%s]",cmd.c_str());

    if(cmd == "cancel"){
        if (base_action_server != NULL){
            base_action_server->bBaseRun = false;
            std::cout<< "bRun is set to false" << std::endl;
        }else{
            std::cout<< "Action_server::subscriber_cb is NULL" << std::endl;
        }
        as_.setAborted(result_);
        ROS_INFO("cancel current action");
    }


}




}