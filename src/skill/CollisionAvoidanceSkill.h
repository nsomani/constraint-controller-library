//
//Copyright (c) 2016, Nikhil Somani
//All rights reserved.
//
//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions are met:
//
//* Redistributions of source code must retain the above copyright notice,
//  this list of conditions and the following disclaimer.
//* Redistributions in binary form must reproduce the above copyright notice,
//  this list of conditions and the following disclaimer in the documentation
//  and/or other materials provided with the distribution.
//
//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
//LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//POSSIBILITY OF SUCH DAMAGE.
//


#ifndef COLLISIONAVOIDANCESKILL_H
#define COLLISIONAVOIDANCESKILL_H

#include "ConstraintSkill.h"

class CollisionAvoidanceSkill : public ConstraintSkill
{
public:
    CollisionAvoidanceSkill();
    CollisionAvoidanceSkill(rl::mdl::Kinematic* &kinematic, std::shared_ptr< rl::sg::Scene > &collisionScene, ObjectModel* &robot, std::vector<rl::sg::Body*> &collision_bodies);
    void updateSkillParams(Eigen::Matrix4d robotPose, Eigen::VectorXd q_current);
    ObjectModel *shape_scene;
    CollisionConstraintPosition *constraint_collision;
private:
    void updatePSPose(PrimitiveShapeDescriptor *ps, Eigen::Matrix4d pose);

};

class CollisionAvoidanceSkillExact : public ConstraintSkill
{
public:
    CollisionAvoidanceSkillExact();
    CollisionAvoidanceSkillExact(rl::mdl::Kinematic* &kinematic, std::shared_ptr< rl::sg::Scene > &collisionScene, ObjectModel* &robot, std::vector<rl::sg::Body*> &collision_bodies);
    void updateSkillParams(Eigen::Matrix4d robotPose, Eigen::VectorXd q_current);
    ObjectModel *shape_scene;
    CollisionConstraintExact *constraint_collision;
private:
    void updatePSPose(PrimitiveShapeDescriptor *ps, Eigen::Matrix4d pose);

};

#endif // COLLISIONAVOIDANCESKILL_H
