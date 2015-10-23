#ifndef BASE_J_ACTION_H_
#define BASE_J_ACTION_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

class Base_j_action{

    enum KUKA_PARAM {
        KUKA_NUM_JOINTS = 7
    };

public:

    Base_j_action(ros::NodeHandle&   nh,
                  const std::string& j_state_pose_topic,
                  const std::string& j_cmd_pos_topic);


   void jStateCallback(const sensor_msgs::JointStateConstPtr& msg);

   void sendJointPos(const Eigen::VectorXd& j_pose);

   void sendJointVel(const Eigen::VectorXd& j_vel);


public:

    ros::Subscriber             sub;
    ros::Publisher              pub;


    sensor_msgs::JointState     j_state;

    Eigen::VectorXd             j_pose;
    Eigen::VectorXd             j_velocity;
    Eigen::VectorXd             j_effort;

};



#endif
