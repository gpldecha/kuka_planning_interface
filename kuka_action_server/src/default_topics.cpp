#include "kuka_action_server/default_topics.h"

/// Cartesian default topics
const std::string topics::EE_STATE_POSE_TOPIC   = "/joint_to_cart/est_ee_pose";
const std::string topics::EE_STATE_FT_TOPIC     = "/joint_to_cart/est_ee_ft";
const std::string topics::EE_CMD_POSE_TOPIC     = "/cart_to_joint/des_ee_pose";
const std::string topics::EE_CMD_FT_TOPIC       = "/cart_to_joint/des_ee_ft";
const std::string topics::BASE_LINK             = "/base_link";

/// Joint default topics
const std::string topics::J_STATE_POSE_TOPIC    =  "/joint_states";
const std::string topics::J_CMD_POSE_TOPIC      =  "/KUKA/joint_cmd";