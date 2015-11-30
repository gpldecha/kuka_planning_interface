#ifndef KUKA_ACTION_SERVER_BASE_J_ACTION_H_
#define KUKA_ACTION_SERVER_BASE_J_ACTION_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <kuka_fri_bridge/JointStateImpedance.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace asrv{

class Base_j_action{

public:

    enum KUKA_PARAM {
        KUKA_NUM_JOINTS = 7
    };

public:

    Base_j_action(ros::NodeHandle&   nh,
                  const std::string& j_state_pose_topic,
                  const std::string& j_cmd_pos_topic,
                  const std::string& j_state_imp_topic,
                  const std::string& j_imp_cmd_topic,
                  const std::string& j_action_topic
                  );


   void jStateCallback(const sensor_msgs::JointStateConstPtr& msg);

   void jStateImpedanceCallback(const kuka_fri_bridge::JointStateImpedanceConstPtr& msg);

   void setJointPos(const Eigen::VectorXd& j_pose);

   void setJointVel(const Eigen::VectorXd& j_vel);

   void sendJointState();

   void sendJointImpedance(const Eigen::VectorXd& j_stiffness);

   void sendJointImpedanceOnly(const Eigen::VectorXd& j_stiffness);

public:

    ros::Subscriber                         sub;
    ros::Publisher                          pub;
  //  ros::Publisher                          pub_ja;

    ros::Subscriber                         sub_imp;
    ros::Publisher                          pub_imp;

    sensor_msgs::JointState                 j_state;
    kuka_fri_bridge::JointStateImpedance    j_state_imp;

    Eigen::VectorXd                         j_position;
    Eigen::VectorXd                         j_velocity;
    Eigen::VectorXd                         j_effort;
    Eigen::VectorXd                         j_stiffness;

private:

    kuka_fri_bridge::JointStateImpedance j_imp_msg;
    sensor_msgs::JointState              j_msg;



};

}

#endif
