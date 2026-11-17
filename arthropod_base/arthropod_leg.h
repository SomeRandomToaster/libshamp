#ifndef ARTHROPOD_LEG_H
#define ARTHROPOD_LEG_H

#include "arthropod_base/arthropod_components.h"
#include "common/joint.h"
#include "geometry/geometry.h"

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

        geometry::Transformation zero_stance()
        {
            geometry::Transformation zero_stance;

            float base_radius = sqrtf(coxa.x() * coxa.x() + coxa.y() * coxa.y());
            float norm_x = coxa.x() / base_radius;
            float norm_y = coxa.y() / base_radius;

            zero_stance.X() = norm_x * gait_config->stance_radius + gait_config->com_x_translation;
            zero_stance.Y() = norm_y * gait_config->stance_radius;
            zero_stance.Z() = coxa.z();
                
            return zero_stance;
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