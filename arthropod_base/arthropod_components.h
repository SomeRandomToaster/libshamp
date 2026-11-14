#ifndef ARTHROPOD_COMPONENTS_H
#define ARTHROPOD_COMPONENTS_H

#include "shamp/common/common_components.h"

namespace shamp
{

    class ArthropodGaitConfig : public champ::GaitConfigBase
    {
        public:
            ArthropodGaitConfig():
                GaitConfigBase()
            {}
            ArthropodGaitConfig(
                float odom_scaler_val,
                float max_l_x,
                float max_l_y,
                float max_a_z,
                float com_x_trans,
                float swing_h,
                float stan_dep,
                float stan_dur,
                float nom_height):
                    GaitConfigBase(
                        odom_scaler_val,
                        max_l_x,
                        max_l_y,
                        max_a_z,
                        com_x_trans,
                        swing_h,
                        stan_dep,
                        stan_dur,
                        nom_height
                    )
            {}
    };
}

#endif