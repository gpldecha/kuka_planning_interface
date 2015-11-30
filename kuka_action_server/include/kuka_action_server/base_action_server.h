#ifndef KUKA_ACTION_SERVER_BASE_ACTION_SERVER_H_
#define KUKA_ACTION_SERVER_BASE_ACTION_SERVER_H_

#include "kuka_action_server/default_services.h"
#include "kuka_action_server/default_types.h"

namespace asrv{


/**
 * @brief The Base_action_server class : Interface to be implemented by user. Each
 * control policy, search method, robot motion heuristic, etc.. has to inherit
 * The Base_action_server class.
 */
class Base_action_server{

public:

    Base_action_server(ros::NodeHandle& nh){
        state_transformer_service = nh.serviceClient<state_transformers::String_cmd>("/state_transformer/cmd");
    }

    /**
     * @brief execute_CB : server action callbac, this function holds the implementation of the
     *                     robots control policy. In this function the user has to send control
     *                     information to the robot.
     * @param as_        : reference to the action server
     */
    virtual bool execute_CB(alib_server& as_,alib_feedback& feedback,const cptrGoal& goal) = 0;


    /**
     * @brief set_control_type : sets the control type [position,velocity] on the state transformer
     * @param ctrl_type: [position,velocity]
     */
    void set_control_type(CTRL_TYPE ctrl_type){
        asrv::set_control_type(ctrl_type,state_transformer_service);
    }

private:

    /**
     * @brief state_transformer_service : used to set the type of control mode in the state transformer
     *                                     type       String_cmd.cmd
     *                                    -------------------------
     *                                    position: | "ctrl position"
     *                                    velocity: | "ctrl velocity"
     */
    ros::ServiceClient   state_transformer_service;


};

}


#endif
