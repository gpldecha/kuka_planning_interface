#ifndef POUR_CLIENT_H_
#define POUR_CLIENT_H_

#include "kuka_action_client/kuka_action_client.h"

class Pour_client : public kac::Kuka_action_client {

public:

        Pour_client(const std::string& name);

};


#endif
