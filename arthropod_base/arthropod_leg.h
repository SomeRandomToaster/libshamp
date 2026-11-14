#ifndef ARTHROPOD_LEG_H
#define ARTHROPOD_LEG_H

#include "shamp/arthropod_base/arthropod_components.h"
#include "shamp/common/joint.h"

namespace shamp
{
    class ArthropodLeg
    {
        unsigned id_;
    public:
        ArthropodLeg():
            id_(0)
        {
            int no_of_joints = 0;
            joint_chain[no_of_joints++] = &coxa;
            joint_chain[no_of_joints++] = &tibia;
            joint_chain[no_of_joints++] = &femur;
            joint_chain[no_of_joints++] = &endpoint;
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

        champ::Joint *joint_chain[4];
    };
}

#endif