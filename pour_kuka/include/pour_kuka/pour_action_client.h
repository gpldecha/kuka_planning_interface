#ifndef KUKA_POUR_CLIENT_H_
#define KUKKA_POUR_CLIENT_H_

#include "kuka_action_client/kuka_action_client.h"

class Create_pour_goals {

public:

        Create_pour_goals();

        std::map<std::string,ac::Goal> pour_goals;


};


#endif
