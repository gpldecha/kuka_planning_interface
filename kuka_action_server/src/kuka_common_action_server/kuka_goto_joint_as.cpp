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
    joint_target_pos.resize(7);
    joint_current_pos.resize(7);
    joint_direction_pos.resize(7);
    des_j_pose.resize(7);

}

bool Kuka_goto_joint_as::execute_CB(alib_server& as_,alib_feedback& feedback_,const cptrGoal& goal){

    std::cout<< "Kuka_goto_joint_as::execute_CB" << std::endl;


    if (action_name == (goal->action_name))
    {
        std::cout<< "- 1 -" << std::endl;


        joint_target_pos[0] =  goal->JointStateImpedance.position[0];
        joint_target_pos[1] =  goal->JointStateImpedance.position[1];
        joint_target_pos[2] =  goal->JointStateImpedance.position[2];
        joint_target_pos[3] =  goal->JointStateImpedance.position[3];
        joint_target_pos[4] =  goal->JointStateImpedance.position[4];
        joint_target_pos[5] =  goal->JointStateImpedance.position[5];
        joint_target_pos[6] =  goal->JointStateImpedance.position[6];


        std::cout<< "- 2 -" << std::endl;


        joint_current_pos = j_pose;

        ROS_INFO("Execution started");
        if (bBaseRun){
            std::cout<< "bRun: TRUE " << std::endl;
        }else{
            std::cout<< "bRun: FALSE " << std::endl;
        }

        double dist_target = 1;

        // Get current joint stiffness



        ros::Duration loop_rate(model_dt);
        while(ros::ok() && bBaseRun) {


            joint_current_pos   = j_pose;
            joint_direction_pos = (joint_target_pos - joint_current_pos);

            ROS_INFO("current: %f %f %f %f %f %f %f",joint_current_pos[0],joint_current_pos[1],joint_current_pos[2],joint_current_pos[3],joint_current_pos[4],joint_current_pos[5],joint_current_pos[6]);
            ROS_INFO("target:  %f %f %f %f %f %f %f",joint_target_pos[0],joint_target_pos[1],joint_target_pos[2],joint_target_pos[3],joint_target_pos[4],joint_target_pos[5],joint_target_pos[6]);
            ROS_INFO("direct:  %f %f %f %f %f %f %f",joint_direction_pos[0],joint_direction_pos[1],joint_direction_pos[2],joint_direction_pos[3],joint_direction_pos[4],joint_direction_pos[5],joint_direction_pos[6]);
            ROS_INFO(" ");

            dist_target = joint_direction_pos.norm();


           // set target joint position
            setJointPos(joint_target_pos);

           // send target joint position + current joint stiffness
            sendJointImpedance(j_stiffness);

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
