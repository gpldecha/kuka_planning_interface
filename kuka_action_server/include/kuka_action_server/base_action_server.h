#ifndef KUKA_ACTION_SERVER_BASE_ACTION_SERVER_H_
#define KUKA_ACTION_SERVER_BASE_ACTION_SERVER_H_

#include "kuka_action_server/default_types.h"

namespace asrv{


/**
 * @brief The Base_action_server class : Interface to be implemented by user. Each
 * control policy, search method, robot motion heuristic, etc.. has to inherit
 * The Base_action_server class.
 */
class Base_action_server{

public:

    Base_action_server(){
        bBaseRun = false;
    }

    /**
     * @brief execute_CB : server action callbac, this function holds the implementation of the
     *                     robots control policy. In this function the user has to send control
     *                     information to the robot.
     * @param as_        : reference to the action server
     */
    virtual bool execute_CB(alib_server& as_,alib_feedback& feedback,const cptrGoal& goal) = 0;

public:

    /**
    * @brief bBaseRun : set to false the execute_CB will terminate and return, true will allow
    *                   execute_CB to run.
    */
   volatile bool bBaseRun;

};

}


#endif
