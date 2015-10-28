#include "kuka_action_client/kuka_action_client.h"
#include <array>

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

void Kuka_action_client::add_default_actions(){

    enum{KUKA_DOF = 7};

    std::array<double,KUKA_DOF> des_velocity;
    std::array<double,KUKA_DOF> des_stiffness;

    kuka_fri_bridge::JointStateImpedance jointStateImpedance;
    jointStateImpedance.position.resize(KUKA_DOF);
    jointStateImpedance.velocity.resize(KUKA_DOF);
    jointStateImpedance.effort.resize(KUKA_DOF);
    jointStateImpedance.stiffness.resize(KUKA_DOF);

    ///--- Gravity Compensation Actions ---///
    {
        kac::Goal goal;
        des_velocity  =  {{0,0,0,0,0,0,0}};
        des_stiffness =  {{20,20,20,20,20,20,20}};

        for(std::size_t i = 0; i < KUKA_DOF;i++){
            jointStateImpedance.velocity[i]      = des_velocity[i];
            jointStateImpedance.stiffness[i]     = des_stiffness[i];
        }

        goal.action_name            = "grav_comp";
        goal.action_type            = "velocity";
        goal.JointStateImpedance    = jointStateImpedance;
        goals["to_grav_comp"]       = goal;
    }
    {
        kac::Goal goal;
        des_velocity  =  {{0,0,0,0,0,0,0}};
        des_stiffness =  {{500,500,500,500,500,500,500}};

        for(std::size_t i = 0; i < 7;i++){
            jointStateImpedance.velocity[i]      = des_velocity[i];
            jointStateImpedance.stiffness[i]     = des_stiffness[i];
        }

        goal.action_name            = "grav_comp";
        goal.action_type            = "velocity";
        goal.JointStateImpedance    = jointStateImpedance;
        goals["to_joint_stiff"]     = goal;
    }

}


}
