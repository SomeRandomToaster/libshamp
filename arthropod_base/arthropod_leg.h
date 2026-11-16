#ifndef ARTHROPOD_LEG_H
#define ARTHROPOD_LEG_H

#include "shamp/arthropod_base/arthropod_components.h"
#include "shamp/common/joint.h"

#include <vector>

namespace shamp
{
    class ArthropodLeg
    {
        unsigned id_;
    public:
        ArthropodLeg():
            id_(0)
        {
            joint_chain.push_back(&coxa);
            joint_chain.push_back(&tibia);
            joint_chain.push_back(&femur);
            joint_chain.push_back(&endpoint);
        }

        void setGaitConfig(shamp::ArthropodGaitConfig *gait_conf)
        {
            gait_config = gait_conf;
        }

        void id(unsigned id)
        {
            id_ = id;
        }

        champ::Joint coxa;
        champ::Joint tibia;
        champ::Joint femur;
        champ::Joint endpoint;

        shamp::ArthropodGaitConfig *gait_config;

        std::vector<champ::Joint *> joint_chain;
    };
}

#endif