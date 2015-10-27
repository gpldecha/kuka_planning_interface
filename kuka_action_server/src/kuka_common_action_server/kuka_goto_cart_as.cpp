#include "kuka_common_action_server/kuka_goto_cart_as.h"
#include <functional>

namespace asrv{

Kuka_goto_cart_as::Kuka_goto_cart_as(ros::NodeHandle& nh, const Action_ee_initialiser &action_ee_init)

    : Base_ee_action(nh,action_ee_init.ee_state_pos_topic,action_ee_init.ee_cmd_pos_topic,action_ee_init.ee_cmd_ft_topic)
{

    action_name             = action_ee_init.action_name;
    world_frame             = action_ee_init.world_frame;

    reachingThreshold       = action_ee_init.reachingThreshold;     // [m]
    orientationThreshold    = action_ee_init.orientationThreshold;  // [rad]

    initial_config          = true;
    simulation              = true;
    tf_count                = 0;
    dt                      = 1.0/100.0;
    default_speed           = 0.05; // [m/s]
}

bool Kuka_goto_cart_as::execute_CB(alib_server& as_,alib_feedback& feedback_,const cptrGoal& goal){

   /* ROS_INFO("Kuka_goto_cart_as::execute_CB");
    ROS_INFO("action_name : [%s]",goal->action_name.c_str());
    ROS_INFO("action_type : [%s]",goal->action_type.c_str());
    ROS_INFO("class action_name : [%s]",action_name.c_str());*/


    if (action_name == (goal->action_name))
    {
        return goto_cartesian_closed_loop(as_,feedback_,goal);
    }else{
        std::string msg;
        msg = "Kuka_goto_cart_as::execute_CB: wrong action call, requested: " + goal->action_name + " actual: " + action_name;
        ROS_ERROR("[%s]",msg.c_str());
        return false;
    }
}

bool Kuka_goto_cart_as::goto_cartesian_closed_loop(alib_server& as_,alib_feedback& feedback,const cptrGoal& goal){

  //  ROS_INFO("goto_cartesian_closed_loop");

    tf::Transform trans_att;

    trans_att.setRotation(tf::Quaternion(goal->attractor_frame.rotation.x,goal->attractor_frame.rotation.y,
                                         goal->attractor_frame.rotation.z,goal->attractor_frame.rotation.w));
    trans_att.setOrigin(tf::Vector3(goal->attractor_frame.translation.x, goal->attractor_frame.translation.y,
                                    goal->attractor_frame.translation.z));

    ros::Rate wait(1);
    tf::Vector3     current_origin = ee_pose.getOrigin();
    tf::Quaternion  current_orient = ee_pose.getRotation();


    tf::Vector3    target_pos    = trans_att.getOrigin() + current_origin;
    tf::Quaternion target_orient = trans_att.getRotation();

    ROS_INFO("current_origin (%f %f %f)",current_origin.x(),current_origin.y(),current_origin.z());
    ROS_INFO("current_target (%f %f %f)",target_pos.x(),target_pos.y(),target_pos.z());

    tf::Vector3    velocity;

    double rate;
    if(goal->dt > 0){
        rate = 1.0/(goal->dt);
    }else{
        rate = 1.0/dt;
    }

    double max_speed =  default_speed; // [ms]
    double speed;
    double dist_targ_origin = (target_pos - current_origin).length();
    double dist_targ_target = current_orient.dot(target_orient);
    double max_dist         = dist_targ_origin;
    double slerp_t          = 0.5;
    double beta             = (1.0/10.0);
    double offset           = 0;
    double prev_orient_error  = 0.0; //[rad]

    ROS_INFO("distance_origin_target %f",dist_targ_origin);
    ROS_INFO("distance_orient_target %f",dist_targ_target);
    ROS_INFO("max_dist %f",max_dist);
    ROS_INFO("max_speed %f [m/s]",max_speed);
    ROS_INFO("rate %f",rate);

    ros::Rate loop_rate(rate);
    while(ros::ok() && bBaseRun) {

        current_origin = ee_pose.getOrigin();
        current_orient = ee_pose.getRotation();

        // Linear velocity between start position and target
        velocity = (target_pos - current_origin);

        // compute desired speed (function of distance to goal, bell shapped velocity curve)
        dist_targ_origin = velocity.length(); // [meters]
        speed            = (max_speed * bell_velocity(dist_targ_origin * 100.0,beta,offset));    // convert [m] -> [cm]
        velocity         = speed * velocity.normalize();

        // ROS_INFO("dist %f [m] %f [cm] %f [speed] %f [bell]",dist_targ_origin,dist_targ_origin*100,speed,bell_velocity(dist_targ_origin * 100.0,beta,offset));

        des_ee_pose.setOrigin(velocity + current_origin);

       // Quaternion slerp interpolation between start and final orientation
       // slerp_t =  1 - (dist_targ_origin/max_dist);
       // ROS_INFO("slerp_t: %f",slerp_t);

       // ROS_INFO("c_q %f %f %f %f",current_orient.w(),current_orient.x(),current_orient.y(),current_orient.z());
       // ROS_INFO("t_q %f %f %f %f",target_orient.w(),target_orient.x(),target_orient.y(),target_orient.z());

        des_ee_pose.setRotation( current_orient.slerp(target_orient, slerp_t)      );

       // ROS_INFO("q: %f %f %f %f",des_ee_pose.getRotation().w(),
       // des_ee_pose.getRotation().x(),des_ee_pose.getRotation().y(),des_ee_pose.getRotation().z());

        dist_targ_target = acos(abs(target_orient.dot(current_orient)));
        sendPose(des_ee_pose);

        feedback.progress = 0;
        as_.publishFeedback(feedback);
        if (as_.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("Preempted");
            as_.setPreempted();
            bBaseRun = false;
            break;
        }

        float diff_ori_err = abs(dist_targ_target - prev_orient_error);
        ROS_INFO_STREAM("Distance to target origin: " << dist_targ_origin << " reachingThreshold: " << reachingThreshold );
        ROS_INFO_STREAM("Distance to target orient: " << dist_targ_target << " orientationsThreshold: " << orientationThreshold );
        ROS_INFO_STREAM("Difference in Orientation error: " << diff_ori_err);

        if (( dist_targ_origin < reachingThreshold) && (dist_targ_target < orientationThreshold || std::isnan(dist_targ_target) || diff_ori_err < 0.001) ){
            ROS_INFO("reached goal");
            break;
        }

        prev_orient_error = dist_targ_target; //[rad]
        loop_rate.sleep();
    }
    if(!bBaseRun){
        return false;
    }else{
        return true;
    }
}

/*
bool Kuka_goto_cart_as::goto_cartesian_open_loop(alib_server& as_,alib_feedback& feedback,const cptrGoal& goal){

    tf::Transform trans_att;

    trans_att.setRotation(tf::Quaternion(goal->attractor_frame.rotation.x,goal->attractor_frame.rotation.y,
                                         goal->attractor_frame.rotation.z,goal->attractor_frame.rotation.w));
    trans_att.setOrigin(tf::Vector3(goal->attractor_frame.translation.x, goal->attractor_frame.translation.y,
                                    goal->attractor_frame.translation.z));

    tf::Quaternion final_orient = trans_att.getRotation();
    tf::Vector3    final_pos    = trans_att.getOrigin();

    tf::Vector3    start_pos    = ee_pose.getOrigin();
    tf::Quaternion start_orient = ee_pose.getRotation();

    double total_time = 2;
    double rate = 100,t=0;
    ros::Rate loop_rate(rate);
    while(ros::ok() && bBaseRun) {

        // Linear interpolation between start and final position
        des_ee_pose.setOrigin(  (1-t/total_time)*start_pos + t/total_time*final_pos  );

        // Quaternion slerp interpolation between start and final orientation
        des_ee_pose.setRotation( start_orient.slerp(final_orient, t/total_time)      );

        t+= 1.0/rate;

        sendPose(des_ee_pose);

        feedback.progress = t;
        as_.publishFeedback(feedback);
        if (as_.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("Preempted");
            as_.setPreempted();
            bBaseRun = false;
            break;
        }

        if(t <= total_time){
            break;
        }
        loop_rate.sleep();
    }
    if(!bBaseRun){
        return false;
    }else{
        return true;
    }
}*/




}
