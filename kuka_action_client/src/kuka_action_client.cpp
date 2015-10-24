#include "kuka_action_client/kuka_action_client.h"

namespace kac{

Kuka_action_client::Kuka_action_client(const std::string& name)
    :ac_(name,true)
{
    ROS_INFO("Waiting for action server to start.");
    ac_.waitForServer(); //will wait for infinite time
    ROS_INFO("Action server started");

    // default initial state, no actions are running
    b_action_running    = false;
    current_action_name = "NONE";
}

void Kuka_action_client::set_goals(std::map<std::string,Goal>& goals){
    this->goals = goals;
}

void Kuka_action_client::push_back(const Goal& goal,const std::string& name){
   goals[name] = goal;
}

bool Kuka_action_client::call_action(const std::string& name){

    std::map<std::string,Goal>::iterator it;

    it = goals.find(name);
    if(it != goals.end()){

        Goal goal               = it->second;
        b_action_running        = true;
        current_action_name     = goal.action_name;

        ac_.sendGoal(it->second);
        ac_.waitForResult();

        actionlib::SimpleClientGoalState state = ac_.getState();

        ROS_INFO("Action finished: %s",state.toString().c_str());
        b_action_running    = false;
        current_action_name = "NONE";
        return true;

    }else{
        std::string msg = "no such action defined: " + name;
        ROS_ERROR("%s",msg.c_str());
        return false;
    }

}


}
