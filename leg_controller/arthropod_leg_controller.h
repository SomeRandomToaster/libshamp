#ifndef ARTHROPOD_LEG_CONTROLLER_H
#define ARTHROPOD_LEG_CONTROLLER_H

#include "arthropod_base/arthropod_base.h"
#include "arthropod_base/arthropod_leg.h"
#include "common/common_components.h"
#include "geometry/geometry.h"
#include "leg_controller/arthropod_phase_generator.h"
#include "leg_controller/arthropod_trajectory_planner.h"

#include <vector>

namespace shamp
{
    class ArthropodLegController
    {
        ArthropodBase* base_;
        std::vector <float> step_lengths_;
        std::vector <float> trajectory_rotations_;
        std::vector <ArthropodTrajectoryPlanner> trajectory_planners_;

        ArthropodPhaseGenerator phase_generator_;

        static float capVelocities(float velocity, float min_velocity, float max_velocity)
        {
            return ((velocity)<(min_velocity)?(min_velocity):((velocity)>(max_velocity)?(max_velocity):(velocity)));
        }

        static float raibertHeuristic (float stance_duration, float target_velocity)
        {
            return (stance_duration / 2.0f) * target_velocity;
        }

        static void transformLeg(float &step_length, float &rotation, ArthropodLeg &leg, float step_x, float step_y, float theta)
        {
            geometry::Transformation transformed_stance = leg.zero_stance();    
            transformed_stance.Translate(step_x, step_y, 0.0f);
            transformed_stance.RotateZ(theta);

            float delta_x = transformed_stance.X() - leg.zero_stance().X();
            float delta_y = transformed_stance.Y() - leg.zero_stance().Y();

            step_length = sqrtf(pow(delta_x, 2) + pow(delta_y, 2));
            rotation = atan2f(delta_y, delta_x);
        }
    public:
        ArthropodLegController(ArthropodBase &base, ArthropodPhaseGenerator::Time time = ArthropodPhaseGenerator::now()):
            base_(&base),
            step_lengths_(base.legs.size()),
            trajectory_rotations_(base.legs.size()),
            phase_generator_(base, time)
        {
            for (size_t i = 0; i < base_->legs.size(); i++) {
                trajectory_planners_.push_back({base_->legs[i]});       
            }
        }

        void velocityCommand(std::vector<geometry::Transformation> &foot_positions, champ::Velocities &req_vel, ArthropodPhaseGenerator::Time time)
        {
            req_vel.linear.x = capVelocities(req_vel.linear.x, -base_->gait_config->max_linear_velocity_x, base_->gait_config->max_linear_velocity_x);
            req_vel.linear.y = capVelocities(req_vel.linear.y, -base_->gait_config->max_linear_velocity_y, base_->gait_config->max_linear_velocity_y);
            req_vel.angular.z = capVelocities(req_vel.angular.z, -base_->gait_config->max_angular_velocity_z, base_->gait_config->max_angular_velocity_z);

            float tangential_velocity = req_vel.angular.z * base_->gait_config->stance_radius;
            float velocity =  sqrtf(pow(req_vel.linear.x, 2) + pow(req_vel.linear.y + tangential_velocity, 2));

            float step_x = raibertHeuristic(base_->gait_config->stance_duration, req_vel.linear.x);
            float step_y = raibertHeuristic(base_->gait_config->stance_duration, req_vel.linear.y);
            float step_theta = raibertHeuristic(base_->gait_config->stance_duration, tangential_velocity);

            float theta = 2 * asinf(step_theta / (2 * base_->gait_config->stance_radius));

            std::fill(step_lengths_.begin(), step_lengths_.end(), 0);
            std::fill(trajectory_rotations_.begin(), trajectory_rotations_.end(), 0);
            float mean_step_len = 0.0f;

            for (size_t i = 0; i < base_->legs.size(); i++) {
                //get the step length and foot trajectory rotation for each leg
                transformLeg(step_lengths_[i], trajectory_rotations_[i], base_->legs[i], step_x, step_y, theta);
                // get the average calculated step lengths since the gait generator has only one parameter for step length
                mean_step_len += step_lengths_[i];
            }
            mean_step_len /= base_->legs.size();

            phase_generator_.run(velocity, time);

            for(size_t i = 0; i < trajectory_planners_.size(); i++)
            {
                trajectory_planners_[i].generate(foot_positions[i], step_lengths_[i], trajectory_rotations_[i], phase_generator_.swing_phase_signal[i], phase_generator_.stance_phase_signal[i]);
            }
        }
    };
}

#endif