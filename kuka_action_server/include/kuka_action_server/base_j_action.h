#ifndef BASE_J_ACTION_H_
#define BASE_J_ACTION_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <kuka_fri_bridge/JointStateImpedance.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

class Base_j_action{

    enum KUKA_PARAM {
        KUKA_NUM_JOINTS = 7
    };

public:

    Base_j_action(ros::NodeHandle&   nh,
                  const std::string& j_state_pose_topic,
                  const std::string& j_cmd_pos_topic,
                  const std::string &j_state_imp_topic,
                  const std::string &j_imp_cmd_topic
                  );


   void jStateCallback(const sensor_msgs::JointStateConstPtr& msg);

   void jStateImpedanceCallback(const kuka_fri_bridge::JointStateImpedanceConstPtr& msg);

   void sendJointPos(const Eigen::VectorXd& j_pose);

   void sendJointVel(const Eigen::VectorXd& j_vel);

   void sendJointImpedance(const Eigen::VectorXd& j_stiffness);

public:

    ros::Subscriber                         sub;
    ros::Publisher                          pub;

    ros::Subscriber                         sub_imp;
    ros::Publisher                          pub_imp;

    sensor_msgs::JointState                 j_state;
    kuka_fri_bridge::JointStateImpedance    j_state_imp;

    Eigen::VectorXd                         j_pose;
    Eigen::VectorXd                         j_velocity;
    Eigen::VectorXd                         j_effort;
    Eigen::VectorXd                         j_stiffness;

};



#endif
