#include "kuka_common_action_server/kuka_goto_cart_as.h"

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
    default_speed           = 0.01; // [m/s]
}

bool Kuka_goto_cart_as::execute_CB(alib_server& as_,alib_feedback& feedback_,const cptrGoal& goal){

    ROS_INFO("Kuka_goto_cart_as::execute_CB");
    ROS_INFO("action_name : [%s]",goal->action_name.c_str());
    ROS_INFO("action_type : [%s]",goal->action_type.c_str());
    ROS_INFO("class action_name : [%s]",action_name.c_str());


    if (action_name == (goal->action_name))
    {
        ROS_INFO("passed action_name == ");

        std::string a_type = goal->action_type;
        if(a_type.size() != 0){

            if(a_type == "open_loop"){
                return goto_cartesian_open_loop(as_,feedback_,goal);

            }else if(a_type == "closed_loop"){
                return goto_cartesian_closed_loop(as_,feedback_,goal);

            }else{
                ROS_ERROR("no such action type [%s], only 'open_loop' or 'closed_loop' actiony type available!",a_type.c_str());
                return false;
            }
        }else{
            return goto_cartesian_open_loop(as_,feedback_,goal);
        }

    }else{
        std::string msg;
        msg = "Kuka_goto_cart_as::execute_CB: wrong action call, requested: " + goal->action_name + " actual: " + action_name;
        ROS_ERROR("[%s]",msg.c_str());
        return false;
    }
}

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
}

bool Kuka_goto_cart_as::goto_cartesian_closed_loop(alib_server& as_,alib_feedback& feedback,const cptrGoal& goal){

    ROS_INFO("goto_cartesian_closed_loop");

    tf::Transform trans_att;

    trans_att.setRotation(tf::Quaternion(goal->attractor_frame.rotation.x,goal->attractor_frame.rotation.y,
                                         goal->attractor_frame.rotation.z,goal->attractor_frame.rotation.w));
    trans_att.setOrigin(tf::Vector3(goal->attractor_frame.translation.x, goal->attractor_frame.translation.y,
                                    goal->attractor_frame.translation.z));

    tf::Quaternion target_orient = trans_att.getRotation();
    tf::Vector3    target_pos    = trans_att.getOrigin();
    tf::Vector3    velocity;

    tf::Vector3    current_origin;
    tf::Quaternion current_orient;

    double rate;
    if(goal->dt > 0){
        rate = 1.0/(goal->dt);
    }else{
        rate = dt;
    }

    double max_speed =  default_speed * (1.0/rate); // [ms]
    double speed;
    double distance_origin_target = (target_pos - current_origin).length();
    double distance_orient_target = current_orient.dot(target_orient);
    double max_dist               = distance_origin_target;
    double slerp_t = 0;

    ROS_INFO("distance_origin_target %f",distance_origin_target);
    ROS_INFO("distance_orient_target %f",distance_orient_target);
    ROS_INFO("max_dist %f",max_dist);

    ros::Rate loop_rate(rate);
    while(ros::ok() && bBaseRun) {

        current_origin = ee_pose.getOrigin();
        current_orient = ee_pose.getRotation();


        // Linear velocity between start position and target
        velocity = (target_pos - current_origin);

        // compute desired speed (function of distance to goal, bell shapped velocity curve)
        distance_origin_target = velocity.length();
        speed                  = gen_logisitic(max_speed,distance_origin_target);
        velocity               = speed * velocity.normalize();

        des_ee_pose.setOrigin(velocity + current_origin);

        // Quaternion slerp interpolation between start and final orientation
        slerp_t =  1 - (distance_origin_target/max_dist);
        des_ee_pose.setRotation( current_orient.slerp(target_orient, slerp_t)      );
        distance_orient_target = current_orient.dot(target_orient);

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

        if ((reachingThreshold >= distance_origin_target) &&
            (orientationThreshold >= distance_orient_target)){
            break;
        }

        loop_rate.sleep();
    }
    if(!bBaseRun){
        return false;
    }else{
        return true;
    }

}


}
