#include "pour_kuka/pour_action_client.h"
#include <geometry_msgs/Transform.h>
#include <Eigen/Core>

Create_pour_goals::Create_pour_goals(){

    enum{KUKA_DOF = 7};

    std::array<double,KUKA_DOF> des_position;
    std::array<double,KUKA_DOF> des_velocity;
    std::array<double,KUKA_DOF> des_stiffness;

    kuka_fri_bridge::JointStateImpedance jointStateImpedance;
    jointStateImpedance.position.resize(KUKA_DOF);
    jointStateImpedance.velocity.resize(KUKA_DOF);
    jointStateImpedance.effort.resize(KUKA_DOF);
    jointStateImpedance.stiffness.resize(KUKA_DOF);



    {   /// HOME action

        // Pouring Object (i.e. dough/plate/etc)
        geometry_msgs::Transform fake_object;
        fake_object.translation.x = 0;
        fake_object.translation.y = 0;
        fake_object.translation.z = 0;
        fake_object.rotation.x    = 0;
        fake_object.rotation.y    = 0;
        fake_object.rotation.z    = 0;
        fake_object.rotation.w    = 1;


        ac::Goal goal;
        goal.action_type = "LEARNED_MODEL";
        goal.object_frame         = fake_object;

        // Good Starting configuration for pouring on LWR LASA
        geometry_msgs::Transform    home;
        home.translation.x = -0.483;
        home.translation.y = 0.091;
        home.translation.z = 0.361;
        home.rotation.w =  0.701;
        home.rotation.x = -0.257;
        home.rotation.y = -0.227;
        home.rotation.z = -0.625;


        goal.action_name           = "home";
        goal.attractor_frame       = home;
        pour_goals[goal.action_name]    = goal;
    }

    {   /// Pouring Phase Attractor

        // Pouring Object (i.e. dough/plate/etc)
        geometry_msgs::Transform fake_object;
        fake_object.translation.x = 0;
        fake_object.translation.y = 0;
        fake_object.translation.z = 0;
        fake_object.rotation.x    = 0;
        fake_object.rotation.y    = 0;
        fake_object.rotation.z    = 0;
        fake_object.rotation.w    = 1;

        ac::Goal goal;
        goal.action_type = "LEARNED_MODEL";
        goal.object_frame         = fake_object;

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
        pour_goals[goal.action_name] = goal;
    }

    {   /// Back Phase attractor

        // Pouring Object (i.e. dough/plate/etc)
        geometry_msgs::Transform fake_object;
        fake_object.translation.x = 0;
        fake_object.translation.y = 0;
        fake_object.translation.z = 0;
        fake_object.rotation.x    = 0;
        fake_object.rotation.y    = 0;
        fake_object.rotation.z    = 0;
        fake_object.rotation.w    = 1;

        ac::Goal goal;
        goal.action_type = "LEARNED_MODEL";
        goal.object_frame         = fake_object;

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
        pour_goals[goal.action_name] = goal;
    }


    // Go to a target joint configuration (INIT)
    {
        ac::Goal goal;
        des_position  =  {{-0.293507303216,-0.268253,0.115399,-1.777066, 0.96409, 1.517602, -0.6515033333075878}};

        for(std::size_t i = 0; i < KUKA_DOF;i++)
            jointStateImpedance.position[i]      = des_position[i];

        goal.action_name            = "goto_joint";
        goal.JointStateImpedance    = jointStateImpedance;
        pour_goals["goto_init"]          = goal;
    }



    // Go to a target joint configuration (HOME)
    {
        ac::Goal goal;
        des_position  =  {{-0.13257421553134918, 0.4764285385608673, -0.5156097412109375, -1.3769710063934326, 0.7565410137176514, 1.7599353790283203, -0.7787759304046631}};

        for(std::size_t i = 0; i < KUKA_DOF;i++)
            jointStateImpedance.position[i]      = des_position[i];

        goal.action_name            = "goto_joint";
        goal.JointStateImpedance    = jointStateImpedance;
        pour_goals["goto_home"]          = goal;
    }

}

