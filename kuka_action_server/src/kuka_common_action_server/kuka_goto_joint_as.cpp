#include "kuka_common_action_server/kuka_goto_joint_as.h"

namespace asrv{


Kuka_goto_joint_as::Kuka_goto_joint_as(ros::NodeHandle&  nh,const Action_j_initialiser &init):
     Base_j_action(nh,
                   init.j_state_pose_topic,
                   init.j_cmd_pos_topic,
                   init.j_imp_topic,
                   init.j_imp_cmd_topic)
{
    action_name = init.action_name;
    model_dt    = init.model_dt;

}

bool Kuka_goto_joint_as::execute_CB(alib_server& as_,alib_feedback& feedback_,const cptrGoal& goal){

    if (action_name == (goal->action_name))
    {


        ROS_INFO("Execution started");
        if (bBaseRun){
            std::cout<< "bRun: TRUE " << std::endl;
        }else{
            std::cout<< "bRun: FALSE " << std::endl;
        }

        ros::Duration loop_rate(model_dt);
        while(ros::ok() && bBaseRun) {



        /*    feedback_.progress = prog_curr;
            as_.publishFeedback(feedback_);
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("Preempted");
                as_.setPreempted();
                bBaseRun = false;
                break;
            }


            if(pos_err < reachingThreshold && (ori_err < orientationThreshold || std::isnan(ori_err))) {
                break;
            }
             */
            loop_rate.sleep();

        }


        if(!bBaseRun){
             return false;
        }else{
             return true;
        }
    }else{
        std::string msg;
        msg = "Kuka_goto_cart_as::execute_CB: wrong action call, requested: " + goal->action_name + " actual: " + action_name;
        ROS_ERROR("[%s]",msg.c_str());
        return false;
    }

}

}
