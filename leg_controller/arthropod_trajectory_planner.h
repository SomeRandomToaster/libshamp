#ifndef ARTHROPOD_TRAJECTORY_PLANNER_H
#define ARTHROPOD_TRAJECTORY_PLANNER_H

#include "arthropod_base/arthropod_leg.h"

#include <vector>

namespace shamp
{
    class ArthropodTrajectoryPlanner
    {
        static constexpr size_t kControlPointsCount = 12;

        ArthropodLeg* leg_;
        float length_ratio_;
        float height_ratio_;
        geometry::Transformation prev_foot_position_;

        std::vector <float> control_points_x_;
        std::vector <float> control_points_y_;

        std::vector <float> ref_control_points_x_;
        std::vector <float> ref_control_points_y_;
        std::vector <float> factorial_;

        bool ran_once_;

        void updateControlPointsHeight(float swing_height)
        {
            float new_height_ratio = swing_height / 0.15f;
            
            if(height_ratio_ != new_height_ratio)
            {
                height_ratio_ = new_height_ratio;
                for(unsigned int i = 0; i < kControlPointsCount; i++)
                {
                    control_points_y_[i] = -((ref_control_points_y_[i] * height_ratio_) + (0.5f * height_ratio_));
                }    
            }
        }

        void updateControlPointsLength(float step_length)
        {
            float new_length_ratio = step_length / 0.4f;
            
            if(length_ratio_ != new_length_ratio)
            {
                length_ratio_ = new_length_ratio;
                for(unsigned int i = 0; i < kControlPointsCount; i++)
                {
                    if(i == 0)
                        control_points_x_[i] = -step_length / 2.0f;
                    else if(i == 11)
                        control_points_x_[i] = step_length / 2.0f;
                    else
                        control_points_x_[i] = ref_control_points_x_[i] * length_ratio_;   
                }
            }
        }
    public:
        ArthropodTrajectoryPlanner(ArthropodLeg &leg):
            leg_(&leg),
            length_ratio_(0),
            height_ratio_(0),
            control_points_x_(kControlPointsCount),
            control_points_y_(kControlPointsCount),
            ref_control_points_x_({-0.15, -0.2805,-0.3,-0.3,-0.3, 0.0, 0.0, 0.0, 0.3032, 0.3032, 0.2826, 0.15}),
            ref_control_points_y_({-0.5, -0.5, -0.3611, -0.3611, -0.3611, -0.3611, -0.3611, -0.3214, -0.3214, -0.3214, -0.5, -0.5}),
            factorial_({1.0,1.0,2.0,6.0,24.0,120.0,720.0,5040.0,40320.0,362880.0,3628800.0,39916800.0,479001600.0}),
            ran_once_(false)
        {}

        void generate(geometry::Transformation &foot_position, float step_length, float rotation, float swing_phase_signal, float stance_phase_signal)
        {
            updateControlPointsHeight(leg_->gait_config->swing_height);

            if(!ran_once_)
            {
                ran_once_ = true;
                prev_foot_position_ = foot_position;                    
            }

            if(step_length == 0.0f)
            {
                prev_foot_position_ = foot_position;
                leg_->gait_phase(1);

                return;
            }

            updateControlPointsLength(step_length);

            int n = kControlPointsCount - 1;
            float x = 0.0f;
            float y = 0.0f;

            if(stance_phase_signal > swing_phase_signal) {
                leg_->gait_phase(1);

                x = (step_length / 2) * (1 - (2 * stance_phase_signal));
                y = -leg_->gait_config->stance_depth * cosf((M_PI * x) / step_length);
            
            } else if(stance_phase_signal < swing_phase_signal) {
                leg_->gait_phase(0);

                for(unsigned int i = 0; i < kControlPointsCount ; i++)
                {
                    float coeff = factorial_[n] / (factorial_[i] * factorial_[n - i]);

                    x += coeff * pow(swing_phase_signal, i) * pow((1 - swing_phase_signal), (n - i)) * control_points_x_[i];
                    y -= coeff * pow(swing_phase_signal, i) * pow((1 - swing_phase_signal), (n - i)) * control_points_y_[i];
                }
                // x = -(step_length / 2) * (1 - (2 * swing_phase_signal));
                // y = leg_->gait_config->swing_height * cosf((M_PI * x) / step_length);
            }

            geometry::Transformation new_position;
            new_position.X() = x * cosf(rotation);
            new_position.Y() = x * sinf(rotation);
            new_position.Z() = y;

            new_position.RotateZ(-leg_->coxa.yaw());

            foot_position.X() += new_position.X();
            foot_position.Y() += new_position.Y();
            foot_position.Z() += new_position.Z();

            if((swing_phase_signal == 0.0f && stance_phase_signal == 0.0f) && step_length > 0.0f)
            {
                foot_position = prev_foot_position_;
            }

            prev_foot_position_ = foot_position;
        }
    };
}

#endif