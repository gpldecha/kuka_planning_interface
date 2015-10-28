#include "pour_kuka/pour_action_client.h"
#include <geometry_msgs/Transform.h>
#include <Eigen/Core>

Pour_client::Pour_client(const std::string& name)
    : kac::Kuka_action_client(name)
{

    enum{KUKA_DOF = 7};

    std::array<double,KUKA_DOF> des_velocity;
    std::array<double,KUKA_DOF> des_stiffness;

    kuka_fri_bridge::JointStateImpedance jointStateImpedance;
    jointStateImpedance.position.resize(KUKA_DOF);
    jointStateImpedance.velocity.resize(KUKA_DOF);
    jointStateImpedance.effort.resize(KUKA_DOF);
    jointStateImpedance.stiffness.resize(KUKA_DOF);


    {   /// HOME action
        kac::Goal goal;
        goal.action_type = "LEARNED_MODEL";
        // Pouring Object (i.e. dough/plate/etc)
        geometry_msgs::Transform fake_object;
        fake_object.translation.x = 0;
        fake_object.translation.y = 0;
        fake_object.translation.z = 0;
        fake_object.rotation.x    = 0;
        fake_object.rotation.y    = 0;
        fake_object.rotation.z    = 0;
        fake_object.rotation.w    = 1;
        goal.object_frame         = fake_object;

        // Good Starting configuration for pouring on LWR LASA
        geometry_msgs::Transform    home;
        home.translation.x         = -0.2;
        home.translation.y         =  0;
        home.translation.z         =  0.2;

        home.rotation.w            =  0;
        home.rotation.x            =  1;
        home.rotation.y            =  0;
        home.rotation.z            =  0;

        goal.action_name           = "home";
        goal.attractor_frame       = home;
        goals[goal.action_name]    = goal;
    }

    {   /// Pouring Phase Attractor
        kac::Goal goal;
        geometry_msgs::Transform pour_attr;
        pour_attr.translation.x  = -0.478;
        pour_attr.translation.y  = -0.184;
        pour_attr.translation.z  =  0.248;
        pour_attr.rotation.x     =  0.133;
        pour_attr.rotation.y     =  0.112;
        pour_attr.rotation.z     = -0.673;
        pour_attr.rotation.w     =  0.719;

        goal.action_name        = "pour";
        goal.attractor_frame    = pour_attr;
        goals[goal.action_name] = goal;
    }

    {   /// Back Phase attractor
        kac::Goal goal;
        geometry_msgs::Transform back_attr;
        back_attr.translation.x = -0.483;
        back_attr.translation.y =  0.091;
        back_attr.translation.z =  0.361;
        back_attr.rotation.w    =  0.701;
        back_attr.rotation.x    = -0.257;
        back_attr.rotation.y    = -0.227;
        back_attr.rotation.z    = -0.625;

        goal.action_name        = "back";
        goal.attractor_frame    = back_attr;
        goals[goal.action_name] = goal;
    }


    {   /// Lock_joint_6
        kac::Goal goal;
        goal.action_name        = "grav_comp";
        goal.action_type        = "velocity";

        // Go to Gravity Compensations with Joint 6 locked
        des_velocity  =  {{0,0,0,0,0,0,0}};
        des_stiffness =  {{0,0,0,0,0,500,0}};

        for(std::size_t i = 0; i < KUKA_DOF;i++){
            jointStateImpedance.velocity[i]      = des_velocity[i];
            jointStateImpedance.stiffness[i]     = des_stiffness[i];
        }
        goal.JointStateImpedance    = jointStateImpedance;
        goals["lock_joint_6"]       = goal;
    }

}

