#ifndef KUKA_ACTION_SERVER_DEFAULT_TYPES_H_
#define KUKA_ACTION_SERVER_DEFAULT_TYPES_H_

#include "actionlib/server/simple_action_server.h"
#include <lasa_action_planners/PLAN2CTRLAction.h>

namespace asrv{

typedef actionlib::SimpleActionServer<lasa_action_planners::PLAN2CTRLAction>    alib_server;
typedef lasa_action_planners::PLAN2CTRLFeedback                                 alib_feedback;
typedef lasa_action_planners::PLAN2CTRLResult                                   alib_result;
typedef lasa_action_planners::PLAN2CTRLGoalConstPtr                             cptrGoal;


}

#endif
