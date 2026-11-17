#ifndef ARTHROPOD_KINEMATICS_H
#define ARTHROPOD_KINEMATICS_H

#include "geometry/geometry.h"
#include "arthropod_base/arthropod_leg.h"

#include <rclcpp/rclcpp.hpp>

namespace shamp
{
    class ArthropodKinematics
    {
        shamp::ArthropodBase *base_;
        std::vector<double> calculated_joints_;
    public:
        ArthropodKinematics(shamp::ArthropodBase &base):
            base_(&base),
            calculated_joints_(base.legs.size() * (base.legs[0].joint_chain.size() - 1)) {}

        static void transformToLocal(geometry::Transformation &foot_position, const shamp::ArthropodLeg &leg)
        {
            foot_position.Translate(-leg.coxa.x(), -leg.coxa.y(), -leg.coxa.z());
            foot_position.RotateZ(-leg.coxa.yaw());
        }

        void inverse(std::vector<double> &joint_positions, std::vector<geometry::Transformation> &foot_positions)
        {
            for(unsigned i = 0; i < base_->legs.size(); i++)
            {
                unsigned joints_per_leg = base_->legs[i].joint_chain.size() - 1;
                inverse(calculated_joints_, i * joints_per_leg, base_->legs[i], foot_positions[i]);
                
                for (unsigned j = 0; j < joints_per_leg; j++) {
                    double position = calculated_joints_[i * joints_per_leg + j];
                    if (isnan(position)) {
                        RCLCPP_WARN(rclcpp::get_logger("kinematics"), "Joint %u position of leg %u is NaN", j, i);
                        return;
                    }
                    if (position < -M_PI_2 || position > M_PI_2) {
                        RCLCPP_WARN(rclcpp::get_logger("kinematics"), "Joint %u position of leg %u is out of bounds: %lf", j, i, M_1_PI * 180 * position);
                        return;
                    }
                }
            }

            for (unsigned i = 0; i < joint_positions.size(); i++) {
                joint_positions[i] = calculated_joints_[i];
            }
        }

        static void inverse(std::vector<double> &joint_positions, size_t first_idx, const shamp::ArthropodLeg&leg, const geometry::Transformation &foot_position)
        {
            geometry::Transformation tmp_position = foot_position;
            float coxa_angle = atan(-tmp_position.Y() / tmp_position.X());

            // Rotate it so the position lies in XZ plane
            tmp_position.RotateZ(coxa_angle);
            tmp_position.Translate(leg.tibia.x(), 0, -leg.tibia.z());
            

            float ik_a1 = sqrtf(leg.femur.x() * leg.femur.x() + leg.femur.y() * leg.femur.y() + leg.femur.z() * leg.femur.z()); // tibia length
            float ik_a2 = sqrtf(leg.endpoint.x() * leg.endpoint.x() + leg.endpoint.y() * leg.endpoint.y() + leg.endpoint.z() * leg.endpoint.z()); // femur length
            float ik_q2 = acos((tmp_position.X() * tmp_position.X() + tmp_position.Z() * tmp_position.Z() - ik_a1 * ik_a1 - ik_a2 * ik_a2) / (2 * ik_a1 * ik_a2));
            float femur_angle = ik_q2 - M_PI_2;

            float ik_beta = atan(ik_a2 * sin(ik_q2) / (ik_a1 + ik_a2 * cos(ik_q2)));
            float ik_gamma = atan(tmp_position.Z() / tmp_position.X());
            float tibia_angle = M_PI_2 - (ik_beta + ik_gamma);

            joint_positions[first_idx] = coxa_angle;
            joint_positions[first_idx + 1] = tibia_angle;
            joint_positions[first_idx + 2] = femur_angle;
        }
    };
}

#endif