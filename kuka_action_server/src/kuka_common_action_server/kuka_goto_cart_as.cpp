#include "kuka_common_action_server/kuka_goto_cart_as.h"
#include <functional>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

namespace asrv{

Kuka_goto_cart_as::Kuka_goto_cart_as(ros::NodeHandle& nh, const Action_ee_initialiser &action_ee_init)
    : Base_ee_action(nh, action_ee_init.ee_state_pos_topic,action_ee_init.ee_cmd_pos_topic,action_ee_init.ee_cmd_ft_topic, action_ee_init.ee_cmd_vel_topic),
      Base_action_server(nh),
      rviz_direction(nh,"kuka_goto_dir"),
      rviz_points_viz(nh,"kuka_goto_target")
{

    action_name             = action_ee_init.action_name;
    world_frame             = action_ee_init.world_frame;

    reachingThreshold       = action_ee_init.reachingThreshold;     // [m]
    orientationThreshold    = action_ee_init.orientationThreshold;  // [rad]


    dt                      = 1.0/100.0;

    rviz_arrow.resize(1);
    rviz_arrow[0].shaft_diameter = 0.005;
    rviz_arrow[0].head_diameter  = 0.01;
    rviz_arrow[0].head_length    = 0.015;

    std::vector<tf::Vector3> colors(1);
    colors[0] = tf::Vector3(1,0,0);

    rviz_direction.initialise(world_frame,rviz_arrow);
    rviz_direction.set_color(colors);
    rviz_direction.scale = 0.05;

    rviz_points.resize(1);

    rviz_points_viz.scale = 0.01;
    rviz_points_viz.r     = 0;
    rviz_points_viz.g     = 1;
    rviz_points_viz.b     = 0;
    rviz_points_viz.initialise(world_frame,rviz_points);

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
    if(action_type == "position"){
        set_control_type(asrv::POSITION);
    }else if(action_type == "velocity"){
        set_control_type(asrv::VELOCITY);
    }

    tf::Vector3    velocity;



    double rate;
    if(goal->dt > 0){
        rate = 1.0/(goal->dt);
    }else{
        rate = 1.0/dt;
    }

    double dist_targ_origin  = (target_pos - current_origin).length();
    double dist_targ_target  = current_orient.dot(target_orient);
    double slerp_t           = 0.5;
    double prev_orient_error = 0.0; //[rad]
    double max_speed         = 0.01;//goal->max_speed;
    double min_speed         = 0.0;//goal->min_speed;
    double speed             = 0.0;


    /// setting bel curve max and min speeds

    if(min_speed <= 0.0){
        max_speed = 0.01;
        ROS_WARN("max_speed not set in goal, max speed set to default: 1cm/s");
    }

    speed_generator.set_max_speed_ms(max_speed);
    speed_generator.set_min_speed_ms(min_speed);


    ROS_INFO("distance_origin_target %f [m]",dist_targ_origin);
    ROS_INFO("distance_orient_target %f [m]",dist_targ_target);
    ROS_INFO("reachingThreashold:    %f [m]",reachingThreshold);
    ROS_INFO("orientationThreshold:  %f [rad]",orientationThreshold);
    ROS_INFO("max_speed              %f",max_speed);
    ROS_INFO("min_speed              %f",min_speed);
    ROS_INFO("rate                   %f",rate);



    static tf::TransformBroadcaster br;
    ros::Rate loop_rate(rate);

    bool success = true;
    tf::Quaternion qdiff;

    while(ros::ok()) {

        br.sendTransform(tf::StampedTransform(trans_att, ros::Time::now(), world_frame, "ee_final"));

        current_origin = ee_pose.getOrigin();
        current_orient = ee_pose.getRotation();

        // Linear velocity between start position and target
        velocity = (target_pos - current_origin);

        // compute desired speed (function of distance to goal, bell shapped velocity curve)
        dist_targ_origin = velocity.length(); // [meters]
        speed_generator.bel_shape_curve(dist_targ_origin);

        speed = speed_generator.bel_shape_curve(dist_targ_origin);

        rviz_arrow[0].origin    = current_origin;
        rviz_arrow[0].direction = 0.05 * velocity.normalize();

        velocity = speed * velocity.normalize();

        des_ee_pose.setOrigin(velocity + current_origin);
        des_ee_pose.setRotation( current_orient.slerp(trans_att.getRotation(), slerp_t));

        rviz_direction.update(rviz_arrow);
        rviz_direction.publish();

        rviz_points[0]  = target_pos;
        rviz_points_viz.update(rviz_points);
        rviz_points_viz.publish();

      //  ROS_INFO("action type: %s",action_type.c_str());

        if(action_type=="position"){
            ROS_INFO("speed: %f",speed);
            ROS_INFO("current_origin (%f %f %f)",current_origin.x(),current_origin.y(),current_origin.z());
            ROS_INFO("current_target (%f %f %f)",target_pos.x(),target_pos.y(),target_pos.z());
            ROS_INFO("current: %f %f %f",current_origin.x(),current_origin.y(),current_origin.z());
            ROS_INFO("desired: %f %f %f",des_ee_pose.getOrigin().getX(),des_ee_pose.getOrigin().getY(),des_ee_pose.getOrigin().getZ());
            sendPose(des_ee_pose);
        }else if (action_type=="velocity"){
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

        //float diff_ori_err = abs(dist_targ_target - prev_orient_error);
        ROS_INFO_STREAM("Distance to target origin: " << dist_targ_origin << " reachingThreshold: " << reachingThreshold );
        ROS_INFO_STREAM("Distance to target orient: " << dist_targ_target << " orientationsThreshold: " << orientationThreshold );
       // ROS_INFO_STREAM("Difference in Orientation error: " << diff_ori_err);


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
