#ifndef KUKA_ACTION_CLIENT_H_
#define KUKA_ACTION_CLIENT_H_

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <lasa_action_planners/PLAN2CTRLAction.h>

#include <map>

namespace kac{

typedef actionlib::SimpleActionClient<lasa_action_planners::PLAN2CTRLAction>    alib_client;
typedef lasa_action_planners::PLAN2CTRLGoal                                     Goal;
typedef actionlib::SimpleClientGoalState::StateEnum                             action_states;

class Kuka_action_client{

public:

    Kuka_action_client(const std::string &name);

    void set_goals(std::map<std::string,Goal>& goals);

    void puch_back(const Goal& goal,const std::string& name);

    bool call_action(const std::string& name);


public:

    alib_client                ac_;
    std::map<std::string,Goal> goals;
    volatile bool              b_action_running;
    std::string                current_action_name;



};

}

#endif
