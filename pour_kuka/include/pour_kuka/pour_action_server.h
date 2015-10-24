#ifndef POUR_ACTION_H_
#define POUR_ACTION_H_

/**
    Example implementation of a pouring task for the KUKA robot. Three types of behaviour
    are implemented; 1) home, 2) back and 3) pour. Which are self explanotary.

  **/

#include "kuka_action_server/action_server.h"
#include "kuka_action_server/base_ee_action.h"
#include "kuka_action_server/default_topics.h"
#include "kuka_action_server/action_server.h"

#include "CDSExecution.h"

class Pour_action_server : public Base_ee_action, public asrv::Base_action_server
{
public:

   enum PouringPhase {
        PHASEHOME=1,
        PHASEPOUR=1,
        PHASEBACK=2
    };

    enum ActionMode {
        ACTION_LASA_FIXED = 1,
        ACTION_ROMEO_FIXED,
        ACTION_VISION
    };

public:

    Pour_action_server(ros::NodeHandle&   nh,
                const std::string& ee_state_pos_topic = topics::EE_STATE_POSE_TOPIC,
                const std::string& ee_cmd_pos_topic   = topics::EE_CMD_POSE_TOPIC,
                const std::string& ee_cmd_ft_topic    = topics::EE_CMD_FT_TOPIC);

    void initialize();

    virtual bool execute_CB(asrv::alib_server& as_,asrv::alib_feedback& feedback,const asrv::cptrGoal& goal);

private:

    bool learned_model_execution(PouringPhase                phase,
                                 asrv::alib_server&          as_,
                                 asrv::alib_feedback&        feedback,
                                 const asrv::cptrGoal&       goal);
private:

    unsigned int    action_mode;
    std::string     world_frame;
    std::string     base_path;
    double          reachingThreshold;
    double          orientationThreshold;
    double          model_dt;
    double          k;
    bool            initial_config;
    bool            simulation;
    int             tf_count;

    tf::Pose        curr_ee_pose;   /// end-effector current position
    tf::Pose        des_ee_pose;    /// desired end-effector position

    CDSController::DynamicsType masterType;
    CDSController::DynamicsType slaveType;
};

#endif

