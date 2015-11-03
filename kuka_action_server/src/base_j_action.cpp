#include "kuka_action_server/base_j_action.h"
#include <assert.h>

namespace asrv{

Base_j_action::Base_j_action(ros::NodeHandle &nh,
                             const std::string &j_state_pose_topic,
                             const std::string &j_cmd_pos_topic,
                             const std::string &j_state_imp_topic,
                             const std::string &j_imp_cmd_topic)
{
    ///-- For Standard Msg Joint State --//
    sub = nh.subscribe<sensor_msgs::JointState>(j_state_pose_topic,1,&Base_j_action::jStateCallback,this);
    pub = nh.advertise<sensor_msgs::JointState>(j_cmd_pos_topic,1);

    ///-- For Custom Msg Joint State Impedance --//
    sub_imp = nh.subscribe<kuka_fri_bridge::JointStateImpedance>(j_state_imp_topic,1,&Base_j_action::jStateImpedanceCallback,this);
    pub_imp = nh.advertise<kuka_fri_bridge::JointStateImpedance>(j_imp_cmd_topic,1);

    ///-- Joint States --//
    j_pose.resize(KUKA_NUM_JOINTS);
    j_velocity.resize(KUKA_NUM_JOINTS);
    j_effort.resize(KUKA_NUM_JOINTS);
    j_stiffness.resize(KUKA_NUM_JOINTS);

}

void Base_j_action::jStateCallback(const sensor_msgs::JointStateConstPtr& msg) {
    const sensor_msgs::JointState* data = msg.get();

     assert(data->position.size() == 7);
     assert(data->velocity.size() == 7);
     assert(data->effort.size()   == 7);

     for(std::size_t i = 0; i < KUKA_NUM_JOINTS;i++){
         j_pose(i)      = data->position[i];
         j_velocity(i)  = data->velocity[i];
         j_effort(i)    = data->effort[i];
     }
}


void Base_j_action::jStateImpedanceCallback(const kuka_fri_bridge::JointStateImpedanceConstPtr& msg){
    const kuka_fri_bridge::JointStateImpedance* data = msg.get();

     assert(data->position.size() == 7);
     assert(data->velocity.size() == 7);
     assert(data->effort.size()   == 7);

     for(std::size_t i = 0; i < KUKA_NUM_JOINTS;i++){
         j_pose(i)      = data->position[i];
         j_velocity(i)  = data->velocity[i];
         j_effort(i)    = data->effort[i];
         j_stiffness(i) = data->stiffness[i];
     }

}

void Base_j_action::setJointPos(const Eigen::VectorXd& j_pose){
    assert(j_pose.size() == KUKA_NUM_JOINTS);

    j_state.position.resize(KUKA_NUM_JOINTS);
    j_state.velocity.resize(0);
    for(std::size_t i = 0; i < KUKA_NUM_JOINTS;i++)
        j_state.position[i] = j_pose[i];

}

void Base_j_action::setJointVel(const Eigen::VectorXd& j_vel){
    assert(j_vel.size() == KUKA_NUM_JOINTS);

    j_state.position.resize(0);
    j_state.velocity.resize(KUKA_NUM_JOINTS);
    for(std::size_t i = 0; i < KUKA_NUM_JOINTS;i++)
        j_state.velocity[i] = j_vel[i];
}

void Base_j_action::sendJointState(){
    sensor_msgs::JointState j_msg;

    j_msg.position.resize(j_state.position.size());
    j_msg.velocity.resize(j_state.velocity.size());

    for(std::size_t i = 0; i < j_state.position.size();i++)
        j_msg.position[i] = j_state.position[i];

    for(std::size_t i = 0; i < j_state.position.size();i++)
        j_msg.velocity[i] = j_state.velocity[i];

    pub.publish(j_msg);

}


void Base_j_action::sendJointImpedance(const Eigen::VectorXd& j_stiff){

    assert(j_stiff.size() == KUKA_NUM_JOINTS);

    kuka_fri_bridge::JointStateImpedance j_imp_msg;
    j_imp_msg.position.resize(j_state.position.size());
    j_imp_msg.velocity.resize(j_state.velocity.size());
    j_imp_msg.stiffness.resize(KUKA_NUM_JOINTS);

    if (j_state.position.size() == KUKA_NUM_JOINTS){

        for(std::size_t i = 0; i < KUKA_NUM_JOINTS;i++){
            j_imp_msg.position[i] = j_state.position[i];
            j_imp_msg.stiffness[i] = j_stiff[i];
        }

    }
    else{

        for(std::size_t i = 0; i < KUKA_NUM_JOINTS;i++){
            j_imp_msg.velocity[i] = j_state.velocity[i];
            j_imp_msg.stiffness[i] = j_stiff[i];
        }


    }

    pub_imp.publish(j_imp_msg);

}

}


