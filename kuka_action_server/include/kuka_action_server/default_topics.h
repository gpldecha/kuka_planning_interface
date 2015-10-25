#ifndef DEFAULT_TOPICS_H_
#define DEFAULT_TOPICS_H_

#include <string>
class topics {

public:

    static const std::string EE_STATE_POSE_TOPIC;
    static const std::string EE_STATE_FT_TOPIC;
    static const std::string EE_CMD_POSE_TOPIC;
    static const std::string EE_CMD_FT_TOPIC ;
    static const std::string BASE_LINK;

    static const std::string J_STATE_POSE_TOPIC;
    static const std::string J_IMP_STATE_TOPIC;
    static const std::string J_CMD_POSE_TOPIC;
    static const std::string J_IMP_CMD_TOPIC;

};



#endif
