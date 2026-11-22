#ifndef ARTHROPOD_PHASE_GENERATOR_H
#define ARTHROPOD_PHASE_GENERATOR_H

#include "arthropod_base/arthropod_base.h"

#include <sys/time.h>
#include <vector>

namespace shamp
{
    class ArthropodPhaseGenerator
    {
    public:
        typedef unsigned long int Time;
        static inline Time now() {
            struct timeval currentTime;
            gettimeofday(&currentTime, NULL);
            return currentTime.tv_sec * (int)1e6 + currentTime.tv_usec;
        }
    private:
        static constexpr size_t kSecondsToMicros = 1e6;
        ArthropodBase* base_;
        std::vector <float> leg_clocks_;

        Time last_touchdown_;
        bool has_swung_;
        bool has_started_;
    public:
        ArthropodPhaseGenerator(ArthropodBase &base, Time time = now()):
            base_(&base),
            leg_clocks_(base.legs.size()),
            last_touchdown_(time),
            has_swung_(false),
            has_started_(false),
            stance_phase_signal(base.legs.size()),
            swing_phase_signal(base.legs.size())
        {}

        void run(float target_velocity, Time time = now())
        {
            unsigned long elapsed_time_ref = 0;
            float swing_phase_period = base_->gait_config->swing_duration * kSecondsToMicros;
            float stance_phase_period =  base_->gait_config->stance_duration * kSecondsToMicros;
            float stride_period = stance_phase_period + swing_phase_period;

            std::fill(leg_clocks_.begin(), leg_clocks_.end(), 0);

            if(target_velocity == 0.0f)
            {
                elapsed_time_ref = 0;
                last_touchdown_ = 0;
                has_swung_ = false;

                std::fill(leg_clocks_.begin(), leg_clocks_.end(), 0);
                std::fill(stance_phase_signal.begin(), stance_phase_signal.end(), 0);
                std::fill(swing_phase_signal.begin(), swing_phase_signal.end(), 0);

                return;
            }

            if(!has_started_)
            {
                has_started_ = true;
                last_touchdown_ = time;
            }

            if((time - last_touchdown_) >= stride_period)
            {
                last_touchdown_ = time;
            }

            if(elapsed_time_ref >= stride_period) {
                elapsed_time_ref = stride_period;
            } else {
                elapsed_time_ref = time - last_touchdown_;
            }
                
            for (size_t i = 0; i < leg_clocks_.size(); i++) {
                leg_clocks_[i] = elapsed_time_ref;
                if (i % 2) {
                    leg_clocks_[i] -= 0.5f * stride_period;
                }
            }

            for(size_t i = 0; i < leg_clocks_.size(); i++)
            {
                if(leg_clocks_[i] > 0 and leg_clocks_[i] < stance_phase_period) {
                    stance_phase_signal[i] = leg_clocks_[i] / stance_phase_period;
                } else {
                    stance_phase_signal[i] = 0;
                }
                        
                if(leg_clocks_[i] > -swing_phase_period && leg_clocks_[i] < 0) {
                    swing_phase_signal[i] = (leg_clocks_[i] + swing_phase_period) / swing_phase_period;
                } else if (leg_clocks_[i] > stance_phase_period && leg_clocks_[i] < stride_period) {
                    swing_phase_signal[i] = (leg_clocks_[i] - stance_phase_period) / swing_phase_period;
                } else {
                    swing_phase_signal[i] = 0;
                }
            }

            if(!has_swung_ && stance_phase_signal[0] < 0.5)
            {
                for (size_t i = 0; i < stance_phase_signal.size(); i++) {
                    if (i % 2) {
                        swing_phase_signal[i] = 0;
                    } else {
                        stance_phase_signal[i] = 0;
                    }
                }
            } else {
                has_swung_ = true;
            }  
        }

        std::vector <float> stance_phase_signal;
        std::vector <float> swing_phase_signal;
    };
}

#endif