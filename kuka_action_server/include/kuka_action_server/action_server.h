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

//-- CDS Stuff --//
//-- Eigen Stuff --//
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <functional>

#include <map>


namespace asrv{

typedef actionlib::SimpleActionServer<lasa_action_planners::PLAN2CTRLAction>    alib_server;
typedef lasa_action_planners::PLAN2CTRLFeedback                                 alib_feedback;
typedef lasa_action_planners::PLAN2CTRLResult                                   alib_result;
typedef lasa_action_planners::PLAN2CTRLGoalConstPtr                             cptrGoal;

/**
 * @brief fexecuteCB, This function should be able to listen to topics and publish
 * to topics such to send and receive state information from the robot or simulation.
 *
 */
typedef std::function<bool(alib_server& as_,alib_feedback& feedback,const cptrGoal& goal)>  fexecuteCB;

class Base_action_server{

public:

    Base_action_server(){
        bBaseRun = false;
    }

    virtual bool execute_CB(alib_server& as_,alib_feedback& feedback,const cptrGoal& goal) = 0;

public:

   volatile bool bBaseRun;


};


class Action_server{

public:

    Action_server(ros::NodeHandle& nh,std::string name);

    void push_back(fexecuteCB& function,std::string action_name);

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

    std::map<std::string,fexecuteCB*>           actions;
    std::map<std::string,fexecuteCB*>::iterator actions_it;


    Base_action_server*                                 base_action_server;
    std::map<std::string,Base_action_server*>           actions2;
    std::map<std::string,Base_action_server*>::iterator actions2_it;

    volatile bool                       *ptr_isOkay;


};

}


#endif
