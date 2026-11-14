#ifndef ARTHROPOD_BASE_H
#define ARTHROPOD_BASE_H

#include "shamp/arthropod_base/arthropod_leg.h"
#include "shamp/common/common_components.h"

#include <vector>

namespace shamp
{
    class ArthropodBase
    {
        champ::Velocities speed_;
    public:
        ArthropodBase(unsigned n_legs):
            legs(n_legs)
        {
            setGaitConfig(gait_config);
        }

        void setGaitConfig(shamp::ArthropodGaitConfig &gait_conf)
        {
            gait_config = gait_conf;

            for(unsigned i = 0; i < legs.size(); i++)
            {
                legs[i].id(i);
                legs[i].setGaitConfig(&gait_config);
            }
        }

        std::vector<shamp::ArthropodLeg> legs;

        shamp::ArthropodGaitConfig gait_config;
    };
}

#endif