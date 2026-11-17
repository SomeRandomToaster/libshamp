#ifndef ARTHROPOD_BODY_CONTROLLER_H
#define ARTHROPOD_BODY_CONTROLLER_H

#include "arthropod_base/arthropod_base.h"
#include "geometry/geometry.h"
#include "kinematics/arthropod_kinematics.h"

#include <vector>

namespace shamp
{
    class ArthropodBodyController
    {
        ArthropodBase *base_;
    public:
        ArthropodBodyController(ArthropodBase& base):
            base_(&base) {}

        void poseCommand(std::vector<geometry::Transformation> &foot_positions, const champ::Pose &req_pose)
        {
            for(size_t i = 0; i < foot_positions.size(); i++)
            {
                    poseCommand(foot_positions[i], base_->legs[i], req_pose);
                }
            }
            
            static void poseCommand(geometry::Transformation &foot_position, 
                                    shamp::ArthropodLeg &leg, 
                                    const champ::Pose &req_pose)
            {
                
                float req_translation_x = -req_pose.position.x + leg.zero_stance().X();
                float req_translation_y = -req_pose.position.y + leg.zero_stance().Y();
                float req_translation_z = -(leg.zero_stance().Z() + req_pose.position.z);

                //create a new foot position
                foot_position = leg.zero_stance();

                //move the foot position to desired body` position of the robot
                foot_position.Translate(req_translation_x, req_translation_y, req_translation_z);

                //rotate the leg opposite the required orientation of the body
                foot_position.RotateZ(-req_pose.orientation.yaw);
                foot_position.RotateY(-req_pose.orientation.pitch);
                foot_position.RotateX(-req_pose.orientation.roll);

                // transform it to leg's local CS 
                shamp::ArthropodKinematics::transformToLocal(foot_position, leg);
                
            }
    };
}

#endif