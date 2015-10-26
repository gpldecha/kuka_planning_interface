#include "pour_kuka/pour_action_client.h"
#include <geometry_msgs/Transform.h>

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

}

