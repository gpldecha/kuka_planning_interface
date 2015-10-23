#ifndef POUR_ACTION_H_
#define POUR_ACTION_H_


#include "kuka_action_server/base_action.h"
#include "kuka_action_server/default_topics.h"
#include "kuka_action_server/action_server.h"

#include "CDSExecution.h"

class Pour_action_server : public Base_action {

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

    bool executeCB(asrv::alib_server& as_, asrv::alib_feedback& feedback_,
                   const lasa_action_planners::PLAN2CTRLGoalConstPtr& goal,
                   PouringPhase phase);

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

    CDSController::DynamicsType masterType;
    CDSController::DynamicsType slaveType;

};

#endif

