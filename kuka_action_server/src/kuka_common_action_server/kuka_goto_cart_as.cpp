#include "kuka_common_action_server/kuka_goto_cart_as.h"
#include <functional>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

namespace asrv{

Kuka_goto_cart_as::Kuka_goto_cart_as(ros::NodeHandle& nh, const Action_ee_initialiser &action_ee_init)

    : Base_ee_action(nh, action_ee_init.ee_state_pos_topic,action_ee_init.ee_cmd_pos_topic,action_ee_init.ee_cmd_ft_topic, action_ee_init.ee_cmd_vel_topic)
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


    tf::Transform trans_att;

    trans_att.setRotation(tf::Quaternion(goal->attractor_frame.rotation.x,goal->attractor_frame.rotation.y,
                                         goal->attractor_frame.rotation.z,goal->attractor_frame.rotation.w));
    trans_att.setOrigin(tf::Vector3(goal->attractor_frame.translation.x, goal->attractor_frame.translation.y,
                                    goal->attractor_frame.translation.z));

    ros::Rate wait(1);
    tf::Vector3     current_origin = ee_pose.getOrigin();
    tf::Quaternion  current_orient = ee_pose.getRotation();


    tf::Vector3    target_pos    = trans_att.getOrigin();
    tf::Quaternion target_orient = trans_att.getRotation();

    ROS_INFO("current_origin (%f %f %f)",current_origin.x(),current_origin.y(),current_origin.z());
    ROS_INFO("current_target (%f %f %f)",target_pos.x(),target_pos.y(),target_pos.z());

    string action_type = goal->action_type;
    ROS_INFO_STREAM("Action Type: " <<  action_type);

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


    static tf::TransformBroadcaster br;
    ros::Rate loop_rate(rate);

    bool success = true;
    tf::Quaternion angvel, qdiff;

    while(ros::ok() /*&& bBaseRun*/) {

        br.sendTransform(tf::StampedTransform(trans_att, ros::Time::now(), world_frame, "ee_final"));

        current_origin = ee_pose.getOrigin();
        current_orient = ee_pose.getRotation();

        // Linear velocity between start position and target
        velocity = (target_pos - current_origin);

        // compute desired speed (function of distance to goal, bell shapped velocity curve)
        dist_targ_origin = velocity.length(); // [meters]
        speed            = (max_speed * bell_velocity(dist_targ_origin * 100.0,beta,offset));    // convert [m] -> [cm]
        velocity         = speed * velocity.normalize();

        des_ee_pose.setOrigin(velocity + current_origin);
        des_ee_pose.setRotation( current_orient.slerp(target_orient, slerp_t));

        if(action_type=="position")
            sendPose(des_ee_pose);

        if (action_type=="velocity"){
            des_ee_vel.linear.x = velocity[0];
            des_ee_vel.linear.y = velocity[1];
            des_ee_vel.linear.z = velocity[2];

            // Computing angular velocity from quaternion differentiation
            qdiff =  des_ee_pose.getRotation() - current_orient;
            Eigen::Vector4f  q  (current_orient.getW(),current_orient.getX(),current_orient.getY(), current_orient.getZ());
            Eigen::Vector4f  dq (qdiff.getW(),qdiff.getX(),qdiff.getY(),qdiff.getZ());
            Eigen::Vector3f  w = d2qw(q,dq);

            des_ee_vel.angular.x = w[0];
            des_ee_vel.angular.y = w[1];
            des_ee_vel.angular.z = w[2];

            sendVel(des_ee_vel);
        }

        dist_targ_target = acos(abs(target_orient.dot(current_orient)));
        feedback.progress = dist_targ_target;
        as_.publishFeedback(feedback);
        if (as_.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("Preempted");
            as_.setPreempted();
            success = false;
            break;
        }

        float diff_ori_err = abs(dist_targ_target - prev_orient_error);
        ROS_INFO_STREAM("Distance to target origin: " << dist_targ_origin << " reachingThreshold: " << reachingThreshold );
        ROS_INFO_STREAM("Distance to target orient: " << dist_targ_target << " orientationsThreshold: " << orientationThreshold );
        ROS_INFO_STREAM("Difference in Orientation error: " << diff_ori_err);

        reachingThreshold = 0.005;
        orientationThreshold = 0.05;
        if (( dist_targ_origin < reachingThreshold) && (dist_targ_target < orientationThreshold || std::isnan(dist_targ_target)) ){
            ROS_INFO("reached goal");
            break;
        }

        prev_orient_error = dist_targ_target; //[rad]
        loop_rate.sleep();
    }

    return success;
}

Eigen::Vector3f Kuka_goto_cart_as::d2qw(Eigen::Vector4f  q,  Eigen::Vector4f  dq){
    // -- DQ2W Converts Quaterion rates to angular velocity --//
    Eigen::Vector3f w;
    double qw = q(0);
    double qx = q(1);
    double qy = q(2);
    double qz = q(3);

    Eigen::MatrixXf W(3,4);
    W << -qx,  qw, -qz,  qy,
         -qy,  qz,  qw, -qx,
         -qz, -qy,  qx,  qw;

    w = 2 * W * dq;

    std::cout << w << std::endl;
    return w;
}





}
