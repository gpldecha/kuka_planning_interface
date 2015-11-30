#include "kuka_action_server/action_server.h"


namespace asrv {

Action_server::Action_server(ros::NodeHandle& nh,std::string name):
    as_(nh, name, boost::bind(&Action_server::executeCB, this, _1), false),
    action_name_(name)
{
    as_.start();

  //  as_sub             = nh.subscribe("/kuka_server/cmd", 10, &Action_server::subscriber_cb,this);
    base_action_server = NULL;

    add_default_actions(nh);

    state_transformer_service = nh.serviceClient<state_transformers::String_cmd>("/state_transformer/cmd");


}

void Action_server::push_back(Base_action_server* base_action_server,std::string action_name){
    actions[action_name] = base_action_server;
}

void Action_server::executeCB(const cptrGoal& goal){


    if(!ros::ok()) {
        result_.success = 0;
        ROS_INFO("%s: Failed", action_name_.c_str());
        as_.setAborted(result_);
        return;
    }

    // initialize action progress as null
    feedback_.progress = 0;

    ///////////////////////////////////////////////
    /////----- EXECUTE REQUESTED ACTION ------/////
    ///////////////////////////////////////////////

    std::string action_type = goal->action_type;
    std::string action_name = goal->action_name;
    std::cout<< "action_type: " << action_type << std::endl;
    std::cout<< "action_name: " << action_name << std::endl;

    actions_it              = actions.find(action_name);


    if(actions_it == actions.end()){
        ROS_ERROR_STREAM("Unidentified action name "<< action_name.c_str());
        result_.success = false;
        as_.setAborted(result_);
    }else{

        base_action_server = actions_it->second;
        bool success       = base_action_server->execute_CB(as_,feedback_,goal);
        result_.success    = success;

        asrv::set_control_type(asrv::NONE,state_transformer_service);
        ros::spinOnce();

        if(success)
        {
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            as_.setSucceeded(result_);
        } else {
            ROS_INFO("%s: Failed", action_name_.c_str());
            as_.setPreempted(result_);
        }
    }
}


void Action_server::add_default_actions(ros::NodeHandle &nh){

    // Grav Comp actions
    {
        asrv::Action_j_initialiser action_j_init;
        action_j_init.action_name = "grav_comp";

        asrv::Action_ee_initialiser action_ee_init;
        action_ee_init.action_name = "grav_comp";

        ptr_kuka_grav_as =  std::shared_ptr<asrv::Kuka_grav_as>( new  asrv::Kuka_grav_as(nh,action_j_init, action_ee_init) );

        push_back(ptr_kuka_grav_as.get(),"grav_comp");
    }

    // Goto joint
    {
         asrv::Action_j_initialiser action_j_init;
         action_j_init.action_name = "goto_joint";


         asrv::Action_ee_initialiser action_ee_init;
         action_ee_init.action_name = "goto_joint";

         ptr_kuka_goto_joint_as = std::shared_ptr<asrv::Kuka_goto_joint_as>(new asrv::Kuka_goto_joint_as(nh,action_j_init,action_ee_init));
         push_back(ptr_kuka_goto_joint_as.get(),"goto_joint");

    }

    // Goto cartesian
    {
         asrv::Action_ee_initialiser action_ee_init;
         action_ee_init.action_name = "goto_cart";
         ptr_kuka_goto_cart_as = std::shared_ptr<asrv::Kuka_goto_cart_as>(new asrv::Kuka_goto_cart_as(nh,action_ee_init));
         push_back(ptr_kuka_goto_cart_as.get(),"goto_cart");
    }

}

void Action_server::print_actions() const{
    std::string action_type;
    std::string action_name;

    std::cout<< "print actions (action server)" << std::endl;
    for(auto it = actions.begin(); it != actions.end(); it++){
        action_name = it->first;
        std::cout<< action_name << std::endl;
    }
    std::cout<<std::endl;

}




}
