#ifndef KUKA_ACTION_SERVER_BASE_EE_ACTION_H_
#define KUKA_ACTION_SERVER_BASE_EE_ACTION_H_

/**
    Base End Effector Action

    Provides a default implementation of ros communicating protocols for the KUKA robot.
    A publisher and subscriber are implemented to read the robot's end effector state
    and command the end effectors state.

  **/

#include <ros/ros.h>
//-- TF Stuff --//
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/WrenchStamped.h"

#include "MathLib/MathLib.h"

#include <Eigen/Core>
#include <Eigen/Geometry>


namespace asrv{

class Base_ee_action{

public:

    Base_ee_action(ros::NodeHandle&   nh,
                const std::string& ee_state_pos_topic,
                const std::string& ee_cmd_pos_topic,
                const std::string& ee_cmd_ft_topic,
                const std::string& ee_cmd_vel_topic);

    void eeStateCallback(const geometry_msgs::PoseStampedConstPtr& msg);

    void sendPose(const tf::Pose& pose_);

    void sendVel(const geometry_msgs::Twist& twist_);

    void toPose(const MathLib::Matrix4& mat4, tf::Pose& pose);

    MathLib::Matrix4 toMatrix4(const tf::Pose& pose);

public:

    ros::Subscriber                     sub_, sub_ft_;
    ros::Publisher                      pub_, pub_ft_, pub_vel_;
    geometry_msgs::PoseStamped          msg_pose;
    geometry_msgs::WrenchStamped        msg_ft;
    geometry_msgs::TwistStamped         msg_vel;

    tf::Pose                            ee_pose;        /// end-effector position from /joint_to_cart estimator
    tf::Pose                            ee_kuka_pose;        /// end-effector position
    Eigen::VectorXd                     ee_ft;          /// end-effector force torque



    volatile bool                       isOkay, isFTOkay;

};

}
#endif
