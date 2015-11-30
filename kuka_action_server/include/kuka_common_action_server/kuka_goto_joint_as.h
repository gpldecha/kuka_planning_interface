#ifndef KUKA_ACTION_SERVER_KUKA_GOTO_JOINT_AS_H_
#define KUKA_ACTION_SERVER_KUKA_GOTO_JOINT_AS_H_

#include "kuka_action_server/default_types.h"
#include "kuka_action_server/base_action_server.h"

#include "kuka_action_server/base_j_action.h"
#include "kuka_action_server/base_ee_action.h"
#include "kuka_common_action_server/action_initialiser.h"

namespace asrv{

class Kuka_goto_joint_as:  public Base_j_action, public Base_ee_action, public Base_action_server {

public:

    Kuka_goto_joint_as(ros::NodeHandle&  nh, const Action_j_initialiser &init, const Action_ee_initialiser &init_ee);

    virtual bool execute_CB(alib_server& as_, alib_feedback& feedback_, const cptrGoal& goal);

private:

    std::string     action_name;
    double          model_dt;
    Eigen::VectorXd joint_target_pos;
    Eigen::VectorXd joint_target_pos_it;
    Eigen::VectorXd joint_target_vel;
    Eigen::VectorXd joint_current_pos;
    Eigen::VectorXd joint_position_error;


    Eigen::VectorXd des_j_pose;
    Eigen::VectorXd des_j_stiffness;


    // Cartesian Commands
    tf::Pose        des_ee_pose_from_curr;    /// desired end-effector position

    kuka_fri_bridge::JointStateImpedance j_imp_msg;


};

}

#endif
