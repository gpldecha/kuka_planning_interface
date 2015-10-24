#include "pour_kuka/pour_client.h"
#include <geometry_msgs/Transform.h>

Pour_client::Pour_client(const std::string& name)
    : kac::Kuka_action_client(name)
{

    kac::Goal goal;
    goal.action_type = "LEARNED_MODEL";

    // Pouring Object (i.e. dough/plate/etc)
    geometry_msgs::Transform pouring_object;
    pouring_object.translation.x = -0.549;
    pouring_object.translation.y = -0.328;
    pouring_object.translation.z = -0.044;
    pouring_object.rotation.x    =  0;
    pouring_object.rotation.y    =  0;
    pouring_object.rotation.z    =  0.999;
    pouring_object.rotation.w    =  0.037;

    goal.object_frame       = pouring_object;


    // Good Starting configuration for pouring on LWR LASA
    geometry_msgs::Transform    home;
    home.translation.x           =  0.2183;
    home.translation.y           = -0.4039;
    home.translation.z           =  0.3678;
    home.rotation.x              = -0.5022;
    home.rotation.y              = -0.6478;
    home.rotation.z              = -0.5465;
    home.rotation.w              =  0.1715;

    goal.action_name        = "home";
    goal.attractor_frame    = home;
    goals[goal.action_name] = goal;

    // Pouring Phase Attractor
    geometry_msgs::Transform pour_attr;
    pour_attr.translation.x =  0.0312;
    pour_attr.translation.y = -0.1720;
    pour_attr.translation.z =  0.3007;
    pour_attr.rotation.x    =  0.3207;
    pour_attr.rotation.y    =  0.8767;
    pour_attr.rotation.z    =  0.2434;
    pour_attr.rotation.w    =  0.2633;

    goal.action_name        = "pour";
    goal.attractor_frame    = pour_attr;
    goals[goal.action_name] = goal;

    // Back Phase Attractor
    geometry_msgs::Transform back_attr;
    back_attr.translation.x = -0.0341;
    back_attr.translation.y = -0.4161;
    back_attr.translation.z =  0.3655;
    back_attr.rotation.x    = -0.4876;
    back_attr.rotation.y    = -0.7555;
    back_attr.rotation.z    = -0.3888;
    back_attr.rotation.w    =  0.2007;

    goal.action_name        = "back";
    goal.attractor_frame    = back_attr;
    goals[goal.action_name] = goal;
}
