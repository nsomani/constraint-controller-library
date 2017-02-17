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


#ifndef CONSTRAINTSKILL_H
#define CONSTRAINTSKILL_H

#include "../RobotControl.h"
#include "../solver/SolverNLOpt.h"
#include "../solver/SolverNLOptGenericKinematic.h"

class ConstraintSkill
{
public:
    ConstraintSkill();
    ConstraintSkill(rl::mdl::Kinematic* kinematic);
    virtual void updateSkillParams(Eigen::Matrix4d robotPose, Eigen::VectorXd q_current);
    virtual bool solveQNLOpt(Eigen::VectorXd &robotTargetConfiguration,std::string solver_name = "COBYLA");
    virtual bool solveDualQNLOpt(Eigen::VectorXd &robotTargetConfiguration);
    void setSolverParams(SolverParams &params);
    std::vector<Constraint*> skill_constraints;
    std::vector<Constraint*> skill_constraints_left;
    std::vector<Constraint*> skill_constraints_right;
    rl::mdl::Kinematic *kinematics;
    Eigen::VectorXd q_current;
    ObjectModel *shape_robot;
protected:
    std::vector<PrimitiveShapeDescriptor*> ps_vector;
    rl::mdl::Kinematic* kinematic1;
    rl::mdl::Kinematic* kinematic2;
    SolverParams solver_params;
};

#endif // CONSTRAINTSKILL_H
