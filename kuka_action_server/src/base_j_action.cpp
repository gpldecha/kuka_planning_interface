#include "kuka_action_server/base_j_action.h"
#include <assert.h>

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

    ///-- Joint Commands --//
    j_state.position.resize(KUKA_NUM_JOINTS);
    j_state.velocity.resize(KUKA_NUM_JOINTS);

    ///-- Joint Impedance Commands --//
    j_state_imp.position.resize(KUKA_NUM_JOINTS);
    j_state_imp.velocity.resize(KUKA_NUM_JOINTS);
    j_state_imp.stiffness.resize(KUKA_NUM_JOINTS);


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

void Base_j_action::sendJointPos(const Eigen::VectorXd& j_pose){
    assert(j_pose.size() == KUKA_NUM_JOINTS);
    for(std::size_t i = 0; i < KUKA_NUM_JOINTS;i++){
        j_state.position[i] = j_pose[i];
        j_state.velocity[i] = 0;

    }
}

void Base_j_action::sendJointVel(const Eigen::VectorXd& j_vel){
    assert(j_vel.size() == KUKA_NUM_JOINTS);
    for(std::size_t i = 0; i < KUKA_NUM_JOINTS;i++){

        j_state.position[i] = 0;
        j_state.velocity[i] = j_vel[i];

    }

}


void Base_j_action::sendJointImpedance(const Eigen::VectorXd& j_stiff){
    assert(j_stiff.size() == KUKA_NUM_JOINTS);
    for(std::size_t i = 0; i < KUKA_NUM_JOINTS;i++){

        j_state_imp.position[i] = j_state.position[i];
        j_state_imp.velocity[i] = j_state.velocity[i];
        j_state_imp.stiffness[i] = j_stiff[i];
    }

}



