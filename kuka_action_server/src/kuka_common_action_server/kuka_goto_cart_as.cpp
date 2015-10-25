#include "kuka_common_action_server/kuka_goto_cart_as.h"

namespace asrv{

Kuka_goto_cart_as::Kuka_goto_cart_as(ros::NodeHandle& nh, const Action_ee_initialiser &action_ee_init)

    : Base_ee_action(nh,action_ee_init.ee_state_pos_topic,action_ee_init.ee_cmd_pos_topic,action_ee_init.ee_cmd_ft_topic)
{

    action_name             = action_ee_init.action_name;
    world_frame             = action_ee_init.world_frame;

    reachingThreshold       = action_ee_init.reachingThreshold;     // [m]
    orientationThreshold    = action_ee_init.orientationThreshold;  // [rad]
    model_dt                = action_ee_init.model_dt;               // [s]

    initial_config          = true;
    simulation              = true;
    tf_count                = 0;
}

bool Kuka_goto_cart_as::execute_CB(alib_server& as_,alib_feedback& feedback_,const cptrGoal& goal){

    if (action_name == (goal->action_name))
    {

        tf::Transform trans_att;

        trans_att.setRotation(tf::Quaternion(goal->attractor_frame.rotation.x,goal->attractor_frame.rotation.y,
                                             goal->attractor_frame.rotation.z,goal->attractor_frame.rotation.w));
        trans_att.setOrigin(tf::Vector3(goal->attractor_frame.translation.x, goal->attractor_frame.translation.y,
                                        goal->attractor_frame.translation.z));

        tf::Transform  trans_final_target;

        if (initial_config == true){
            curr_ee_pose = ee_pose;
        }else{
            curr_ee_pose = des_ee_pose;
        }

        ros::Duration loop_rate(model_dt);
        tf::Pose mNextRobotEEPose = curr_ee_pose;
        tf::Transform trans_ee;
        double pos_err, ori_err, prog_curr;
        pos_err = 0; ori_err = 0; prog_curr = 0;

        ROS_INFO("Execution started");
        if (bBaseRun){
            std::cout<< "bRun: TRUE " << std::endl;
        }else{
            std::cout<< "bRun: FALSE " << std::endl;
        }

        static tf::TransformBroadcaster br;
        while(ros::ok() && bBaseRun) {
            if (initial_config == true)
                curr_ee_pose = ee_pose;
            else
                curr_ee_pose = des_ee_pose;

            // Publish attractors if running in simulation or with fixed values
            trans_ee.setRotation(tf::Quaternion(curr_ee_pose.getRotation()));
            trans_ee.setOrigin(tf::Vector3(curr_ee_pose.getOrigin()));

            // To Visualize EE Frames
            if (simulation==true){
                int frame_viz = int(model_dt*1000);
                if (tf_count==0 || tf_count%frame_viz==0){
                    stringstream ss;
                    ss <<  "/ee_tf_" << tf_count;
                    br.sendTransform(tf::StampedTransform(trans_ee, ros::Time::now(), world_frame, ss.str()));
                }
                tf_count++;
            }
            else{
                br.sendTransform(tf::StampedTransform(trans_ee, ros::Time::now(), world_frame, "/ee_tf"));
            }

            br.sendTransform(tf::StampedTransform(trans_final_target, ros::Time::now(), world_frame, "/attractor"));

            // Current progress variable (position/orientation error).
            // TODO: send this back to action client as current progress
            pos_err = (trans_final_target.getOrigin() - curr_ee_pose.getOrigin()).length();
            ori_err = acos(abs(trans_final_target.getRotation().dot(curr_ee_pose.getRotation())));

            //   double att_pos_err = (trans_final_target.getOrigin() - des_ee_pose.getOrigin()).length();
            double att_ori_err = acos(abs(trans_final_target.getRotation().dot(des_ee_pose.getRotation())));

            // Compute Next Desired EE Pose
            des_ee_pose = mNextRobotEEPose;

            // Make next pose the current pose for open-loop simulation
            if (simulation==true)
                initial_config=false;

            // If orientation error is VERY low or nan because of qdiff take target orientation
            if (att_ori_err < 0.005 || std::isnan(att_ori_err)){ //[rad] and [m]//
                // if (isnan(att_ori_err)) //[rad] and [m]
                des_ee_pose.setRotation(tf::Quaternion(trans_final_target.getRotation()));
            }

            // Send the computed pose from one of the above phases
            if (simulation==false){
                sendPose(des_ee_pose);
            }

            // Broadcast/view Desired EE Pose
            br.sendTransform(tf::StampedTransform(des_ee_pose, ros::Time::now(), world_frame, "/des_ee_mp"));
            // ROS_INFO_STREAM_THROTTLE(1, "Sent Position: "<<des_ee_pose.getOrigin().x()<<","<<des_ee_pose.getOrigin().y()<<","<<des_ee_pose.getOrigin().z());
            tf::Quaternion q = des_ee_pose.getRotation();
            q  = q.normalized();
            // ROS_INFO_STREAM_THROTTLE(1, "Sent Orientation: "<<q.x()<<","<<q.y()<<","<<q.z()<<","<<q.w());

            // ACTION SERVER FUNCTIONALITY

            feedback_.progress = prog_curr;
            as_.publishFeedback(feedback_);
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("Preempted");
                // set the action state to preempted
                as_.setPreempted();
                bBaseRun = false;
                break;
            }

            if(pos_err < reachingThreshold && (ori_err < orientationThreshold || std::isnan(ori_err))) {
                break;
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
