#include "kuka_common_action_server/kuka_goto_joint_as.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>
#include <Eigen/Eigen>

namespace asrv{

Kuka_goto_joint_as::Kuka_goto_joint_as(ros::NodeHandle&  nh,
                           const Action_j_initialiser &init,
                           const Action_ee_initialiser &action_ee_init):
     Base_j_action(nh,
                   init.j_state_pose_topic,
                   init.j_cmd_pos_topic,
                   init.j_imp_topic,
                   init.j_imp_cmd_topic,
                   init.j_action_topic),
     Base_ee_action(nh,
                   action_ee_init.ee_state_pos_topic,
                   action_ee_init.ee_cmd_pos_topic,
                   action_ee_init.ee_cmd_ft_topic)
{
    action_name = init.action_name;
    model_dt    = init.model_dt;
    joint_target_pos.resize(7);
    joint_current_pos.resize(7);
    joint_position_error.resize(7);
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




        ROS_INFO("Execution started");
        if (bBaseRun){
            std::cout<< "bRun: TRUE " << std::endl;
        }else{
            std::cout<< "bRun: FALSE " << std::endl;
        }

        double dist_target(1);
        double max_speed (0.5);
        double p_gain (1.0);
        double dt (0.002);
        double rise_time (1.0);
        double running_time (0.0);

        ros::Duration loop_rate(dt);
        while(ros::ok() && bBaseRun) {


            joint_position_error = (joint_target_pos - j_position);
            dist_target = joint_position_error.norm();

            // Linear velocity between start position and target
            joint_target_vel = joint_position_error*p_gain;

            // If movement just started, take it easy in the beginnning
            if (running_time<rise_time){
                joint_target_vel*=(sin(-PI/2+running_time/rise_time*PI)+1)/2;
            }

            // Clip velocities
            for(size_t i=0; i<7;++i)
                if(joint_target_vel[i] > max_speed)
                    joint_target_vel[i] = max_speed;

            // Compute desired position
            joint_target_pos_it = j_position + joint_target_vel;


           ////--- CHOOSE EITHER POSITION OR VELOCITY CONTROL FOR DIFFERENT SCENARIOS ---///
           // set target joint position (if you want to use this one with real robot
           // you have to increase the stiffness to at least 500 before the transition otherwise CLICK!)
//            setJointPos(joint_target_pos_it);

            // set target joint velocity (this one works smoothly with the real robot and simulation)
            setJointVel(joint_target_vel);

           // send target joint position + current joint stiffness
            sendJointImpedance(j_stiffness);


            ROS_INFO("current: %f %f %f %f %f %f %f",j_position[0],j_position[1],j_position[2],j_position[3],j_position[4],j_position[5],j_position[6]);
            ROS_INFO("target_vel:  %f %f %f %f %f %f %f",joint_target_vel[0],joint_target_vel[1],joint_target_vel[2],joint_target_vel[3],joint_target_vel[4],joint_target_vel[5],joint_target_vel[6]);
            ROS_INFO("target_pos:  %f %f %f %f %f %f %f",joint_target_pos_it[0],joint_target_pos_it[1],joint_target_pos_it[2],joint_target_pos_it[3],joint_target_pos_it[4],joint_target_pos_it[5],joint_target_pos_it[6]);


            ///--- Stop sending when the desired joint position is reached ---///
            ROS_INFO_STREAM("Current error to desired target: " << dist_target);

            if(dist_target < 1e-2){
                ROS_INFO_STREAM('Desired joint position REACHED!');

                //---  Update Cartesian Pose for (cart_to_joint) once you have reached it
                tf::TransformListener listener;
                tf::StampedTransform curr_ee_tf_;
                try{
                    listener.waitForTransform("/world_frame", "/curr_ee_tf",  ros::Time(0), ros::Duration(10.0) );
                    listener.lookupTransform("/world_frame", "/curr_ee_tf",  ros::Time(0), curr_ee_tf_);
                } catch (tf::TransformException ex) {
                    ROS_ERROR("%s",ex.what());
                }

                ROS_INFO_STREAM("Curr_ee_tf x:" <<curr_ee_tf_.getOrigin().x() << " y:" << curr_ee_tf_.getOrigin().y() << " z:" << curr_ee_tf_.getOrigin().z() );

                des_ee_pose_from_curr.setOrigin(curr_ee_tf_.getOrigin());
                des_ee_pose_from_curr.setRotation(curr_ee_tf_.getRotation());
                sendPose(des_ee_pose_from_curr);

                ///-- Publish joint action message (for cart_to_joint) --//
                std_msgs::Bool j_action_msg;
                j_action_msg.data = false;
                pub_ja.publish(j_action_msg);

                return true;
            }

            running_time+=dt;
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
