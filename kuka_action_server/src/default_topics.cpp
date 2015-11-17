#include "kuka_action_server/default_topics.h"
namespace asrv{

/// Cartesian default topics
const std::string topics::EE_KUKA_POSE_TOPIC    = "/KUKA/Pose";
const std::string topics::EE_STATE_POSE_TOPIC   = "/joint_to_cart/est_ee_pose";
const std::string topics::EE_STATE_FT_TOPIC     = "/joint_to_cart/est_ee_ft";

/// Go to state-transformer
const std::string topics::EE_CMD_POSE_TOPIC     = "/cart_to_joint/des_ee_pose";
const std::string topics::EE_CMD_FT_TOPIC       = "/cart_to_joint/des_ee_ft";
const std::string topics::EE_CMD_STIFF_TOPIC    = "/";

const std::string topics::BASE_LINK             = "/base_link";

/// Joint default topics (directly via bridge to the robot, have to take care of smoothing)
const std::string topics::J_STATE_POSE_TOPIC    =  "/joint_states";
const std::string topics::J_IMP_STATE_TOPIC     =  "/joint_imp_states";
const std::string topics::J_CMD_POSE_TOPIC      =  "/KUKA/joint_cmd";
const std::string topics::J_IMP_CMD_TOPIC       =  "/KUKA/joint_imp_cmd";
const std::string topics::J_ACTION_TOPIC       =  "/joint_action";

}
