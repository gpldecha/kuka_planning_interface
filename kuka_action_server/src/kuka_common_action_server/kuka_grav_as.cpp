#include "kuka_common_action_server/kuka_grav_as.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>
#include <Eigen/Eigen>

namespace asrv{


Kuka_grav_as::Kuka_grav_as(ros::NodeHandle&  nh,
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
}


bool Kuka_grav_as::execute_CB(alib_server& as_,alib_feedback& feedback_,const cptrGoal& goal){

    if (action_name == (goal->action_name))
    {

        ROS_INFO("Kuka_grav_as::execute_CB");
        ROS_INFO("action_name : [%s]",goal->action_name.c_str());
        ROS_INFO("action_type : [%s]",goal->action_type.c_str());
        ROS_INFO("desired_stifness: [%f,%f,%f,%f,%f,%f,%f]", goal->JointStateImpedance.stiffness[0],
                goal->JointStateImpedance.stiffness[1],goal->JointStateImpedance.stiffness[2],
                goal->JointStateImpedance.stiffness[3],goal->JointStateImpedance.stiffness[4],
                goal->JointStateImpedance.stiffness[5],goal->JointStateImpedance.stiffness[6]);

        if (bBaseRun){
            std::cout<< "bRun: TRUE " << std::endl;
        }else{
            std::cout<< "bRun: FALSE " << std::endl;
        }

        ///--- Desired Joint Impedance Command--///
        des_j_pose.resize(7);
        des_j_velocity.resize(7);
        des_j_stiffness.resize(7);

        for(std::size_t i = 0; i < 7;++i){
            if (action_type == "position")
                des_j_pose(i)      = goal->JointStateImpedance.position[i];
            else
                des_j_velocity(i)  = goal->JointStateImpedance.velocity[i];

             des_j_stiffness(i) = goal->JointStateImpedance.stiffness[i];
        }


        ///--- Action Type (Control interface: Velocity or Position) ---///
        action_type = goal->action_type;

        ///--- Desired Loop Rate Command--///
        ros::Duration loop_rate(goal->dt);

        while(ros::ok() && bBaseRun) {

            //---  Update Cartesian Pose for (cart_to_joint) everytime you go into interactive grav_comp
            //---  Send /cart_to_joint/des_ee_pos  as the current ee_pose
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


            ///--- Send Joint Impedance Command to KUKA FRI Bridge ---///
            if (action_type == "position")
                setJointPos(des_j_pose);
            else{

                //  Send Velocity command ( will be 0 for grav comp)
                setJointVel(des_j_velocity);

            }
            sendJointImpedance(des_j_stiffness);

            ///--- Stop sending when the desired stiffness is reached ---///
            Eigen::VectorXd stiff_diff = j_stiffness - des_j_stiffness;
            double stiff_err =  abs(stiff_diff.sum());

            ROS_INFO_STREAM("Current error to desired stiffness: " << stiff_err);
            if(stiff_err < 0.5){
                ROS_INFO_STREAM('Desired Stiffness REACHED, you can manipulate the robot now...');

                ///-- Publish joint action message (for cart_to_joint) --//
                std_msgs::Bool j_action_msg;
                j_action_msg.data = false;
                pub_ja.publish(j_action_msg);

                return true;
            }
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
