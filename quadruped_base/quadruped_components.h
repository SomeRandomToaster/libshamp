/*
Copyright (c) 2019-2020, Juan Miguel Jimeno
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef QUADRUPED_COMPONENTS_H
#define QUADRUPED_COMPONENTS_H

#include <shamp/common/common_components.h>

namespace champ
{

    class GaitConfig : public GaitConfigBase
    {
        public:
            GaitConfig():
                GaitConfigBase(),
                knee_orientation(">>"),
                pantograph_leg(false)
            {}
            GaitConfig(const char * knee_or,
                bool panto_leg,
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
                    ),
                    knee_orientation(knee_or),
                    pantograph_leg(panto_leg)
            {}
            const char * knee_orientation;
            bool pantograph_leg;
    };
}

#endif