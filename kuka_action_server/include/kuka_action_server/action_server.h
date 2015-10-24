#ifndef ACTION_SERVER_H_
#define ACTION_SERVER_H_

/**
    Action Server

    This class acts as the main register for indiviudal implemenations of actions



  **/

#include <ros/ros.h>

//-- TF Stuff --//
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

//-- Custom ActionLib Stuff --//
#include "actionlib/server/simple_action_server.h"
#include <lasa_action_planners/PLAN2CTRLAction.h>

//-- Message Types --//
#include <robohow_common_msgs/MotionPhase.h>
#include <robohow_common_msgs/MotionModel.h>
#include <robohow_common_msgs/GaussianMixtureModel.h>
#include <robohow_common_msgs/GaussianDistribution.h>

#include <std_msgs/String.h>

#include <functional>

#include <map>


namespace asrv{

typedef actionlib::SimpleActionServer<lasa_action_planners::PLAN2CTRLAction>    alib_server;
typedef lasa_action_planners::PLAN2CTRLFeedback                                 alib_feedback;
typedef lasa_action_planners::PLAN2CTRLResult                                   alib_result;
typedef lasa_action_planners::PLAN2CTRLGoalConstPtr                             cptrGoal;

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

/**
 * @brief The Action_server class : Contains all off the users implementation of verious control
 * polices. When the action server receives a request from the action client to run a particular
 * action, the action server will execute the callback associated with the requested action.
 */

class Action_server{

public:

    Action_server(ros::NodeHandle& nh,std::string name);

    /**
     * @brief push_back             : add's an action implementation with associated reference name
     *                                to the action server.
     * @param base_action_server    : all action implementations inherite "Base_action_server" for which
     *                                the user will have provided his own control policy implementation.
     */
    void push_back(Base_action_server* base_action_server,std::string action_name);

private:

    void executeCB(const cptrGoal &goal);

    void subscriber_cb(const std_msgs::String::ConstPtr& msg);

private:

    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    alib_server                         as_;
    std::string                         action_name_;
    // create messages that are used to published feedback/result
    alib_feedback                       feedback_;
    alib_result                         result_;
    ros::Subscriber                     as_sub;

    Base_action_server*                                 base_action_server;

    /**
     * @brief actions : key is the name of an action and the value is a pointer
     * to a particular control policy.
     */
    std::map<std::string,Base_action_server*>           actions;
    std::map<std::string,Base_action_server*>::iterator actions_it;

};

}


#endif
