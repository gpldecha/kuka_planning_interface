#include "pour_kuka/pour_action_client.h"
#include <geometry_msgs/Transform.h>
#include <Eigen/Core>

Pour_client::Pour_client(const std::string& name)
    : kac::Kuka_action_client(name)
{

    kac::Goal goal;
    goal.action_type = "LEARNED_MODEL";

   // Pouring Object (i.e. dough/plate/etc)
    geometry_msgs::Transform fake_object;
    fake_object.translation.x = 0;
    fake_object.translation.y = 0;
    fake_object.translation.z = 0;
    fake_object.rotation.x = 0;
    fake_object.rotation.y = 0;
    fake_object.rotation.z = 0;
    fake_object.rotation.w = 1;

    goal.object_frame       = fake_object;



    // Good Starting configuration for pouring on LWR LASA
    geometry_msgs::Transform    home;
    home.translation.x = -0.483;
    home.translation.y = 0.091;
    home.translation.z = 0.361;
    home.rotation.w =  0.701;
    home.rotation.x = -0.257;
    home.rotation.y = -0.227;
    home.rotation.z = -0.625;

    goal.action_name        = "home";
    goal.attractor_frame    = home;
    goals[goal.action_name] = goal;

    // Pouring Phase Attractor
    geometry_msgs::Transform pour_attr;
    pour_attr.translation.x  = -0.478;
    pour_attr.translation.y  = -0.184;
    pour_attr.translation.z  = 0.248;
    pour_attr.rotation.x     = 0.133;
    pour_attr.rotation.y     =  0.112;
    pour_attr.rotation.z     = -0.673;
    pour_attr.rotation.w     = 0.719;

    goal.action_name        = "pour";
    goal.attractor_frame    = pour_attr;
    goals[goal.action_name] = goal;

    // Back Phase Attractor
    geometry_msgs::Transform back_attr;
    back_attr.translation.x = -0.483;
    back_attr.translation.y = 0.091;
    back_attr.translation.z = 0.361;
    back_attr.rotation.w    =  0.701;
    back_attr.rotation.x    = -0.257;
    back_attr.rotation.y    = -0.227;
    back_attr.rotation.z    = -0.625;


    goal.action_name        = "back";
    goal.attractor_frame    = back_attr;
    goals[goal.action_name] = goal;

    ///--- GoTo Cartesian Actions ---///

    geometry_msgs::Transform    home2;
    home2.translation.x      = -0.1;
    home2.translation.y      =  0;
    home2.translation.z      =  -0.2;

    home2.rotation.w         =  1;
    home2.rotation.x         =  0;
    home2.rotation.y         =  0;
    home2.rotation.z         =  0;

    kac::Goal goal2;
    goal2.action_name        = "goto_home";
    goal2.attractor_frame    = home2;
    goal2.action_type        = "closed_loop";
    goals[goal2.action_name] = goal2;


    ///--- Gravity Compensation Actions ---///
    kac::Goal goal3;
    kuka_fri_bridge::JointStateImpedance jointStateImpedance;
    jointStateImpedance.position.resize(7);
    jointStateImpedance.velocity.resize(7);
    jointStateImpedance.effort.resize(7);
    jointStateImpedance.stiffness.resize(7);

    Eigen::VectorXd des_velocity;
    Eigen::VectorXd des_stiffness;
    des_velocity.resize(7);
    des_stiffness.resize(7);

    // Go to Gravity Compensation
    des_velocity  << 0,0,0,0,0,0,0;
    des_stiffness << 0,0,0,0,0,0,0;

    for(std::size_t i = 0; i < 7;i++){
        jointStateImpedance.velocity[i]      = des_velocity[i];
        jointStateImpedance.stiffness[i]     = des_stiffness[i];
    }

    goal3.action_name        = "grav_comp";
    goal3.action_type        = "velocity";
    goal3.JointStateImpedance    = jointStateImpedance;
    goals["to_grav_comp"] = goal3;

    // Go Back to Joint Impedance Mode
    des_velocity  << 0,0,0,0,0,0,0;
    des_stiffness << 200,200,200,200,200,200,200;

    for(std::size_t i = 0; i < 7;i++){
        jointStateImpedance.velocity[i]      = des_velocity[i];
        jointStateImpedance.stiffness[i]     = des_stiffness[i];
    }

    goal3.action_name        = "grav_comp";
    goal3.action_type        = "velocity";
    goal3.JointStateImpedance    = jointStateImpedance;
    goals["to_joint_imp"] = goal3;

    // Go Back to Joint Impedance Mode
    des_velocity  << 0,0,0,0,0,0,0;
    des_stiffness << 50,50,50,50,50,50,50;

    for(std::size_t i = 0; i < 7;i++){
        jointStateImpedance.velocity[i]      = des_velocity[i];
        jointStateImpedance.stiffness[i]     = des_stiffness[i];
    }

    goal3.action_name        = "grav_comp";
    goal3.action_type        = "velocity";
    goal3.JointStateImpedance    = jointStateImpedance;
    goals["to_joint_imp_incr"] = goal3;


    // Go Back to Joint Impedance Mode
    des_velocity  << 0,0,0,0,0,0,0;
    des_stiffness << 500,500,500,500,500,500,500;

    for(std::size_t i = 0; i < 7;i++){
        jointStateImpedance.velocity[i]      = des_velocity[i];
        jointStateImpedance.stiffness[i]     = des_stiffness[i];
    }

    goal3.action_name        = "grav_comp";
    goal3.action_type        = "velocity";
    goal3.JointStateImpedance    = jointStateImpedance;
    goals["to_joint_stiff"] = goal3;

    // Go to Gravity Compensations with Joint 6 locked
    des_velocity  << 0,0,0,0,0,0,0;
    des_stiffness << 0,0,0,0,0,500,0;

    for(std::size_t i = 0; i < 7;i++){
        jointStateImpedance.velocity[i]      = des_velocity[i];
        jointStateImpedance.stiffness[i]     = des_stiffness[i];
    }

    goal3.action_name        = "grav_comp";
    goal3.action_type        = "velocity";
    goal3.JointStateImpedance    = jointStateImpedance;
    goals["lock_joint_6"] = goal3;




}

