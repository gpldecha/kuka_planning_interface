#ifndef KUKA_ACTION_SERVER_KUKA_GOTO_JOINT_AS_H_
#define KUKA_ACTION_SERVER_KUKA_GOTO_JOINT_AS_H_

#include "kuka_action_server/base_action_server.h"
#include "kuka_action_server/base_j_action.h"
#include "kuka_common_action_server/action_initialiser.h"

namespace asrv{

class Kuka_goto_joint_as : public Base_j_action, public Base_action_server {

public:

    Kuka_goto_joint_as(ros::NodeHandle&  nh, const Action_j_initialiser &init);

    virtual bool execute_CB(alib_server& as_, alib_feedback& feedback_, const cptrGoal& goal);

private:

    std::string action_name;
    double      model_dt;

};

}

#endif
