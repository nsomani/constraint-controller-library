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


#include "ConstraintSkill.h"

ConstraintSkill::ConstraintSkill()
{
}

ConstraintSkill::ConstraintSkill(rl::mdl::Kinematic *kinematic):
    kinematics(kinematic)
{

}

void ConstraintSkill::updateSkillParams(Eigen::Matrix4d robotPose, Eigen::VectorXd q_current)
{
    this->q_current = q_current;
    shape_robot->transform(robotPose);
}

bool ConstraintSkill::solveQNLOpt(Eigen::VectorXd &robotTargetConfiguration, std::string solver_name)
{
    SolverNLOptGenericKinematic kin_solver_nlopt(skill_constraints, q_current, kinematics, solver_name);
    kin_solver_nlopt.setSolverParams(this->solver_params);
    robotTargetConfiguration = kin_solver_nlopt.solve();
//    KinematicSolverNLOpt kin_solver_nlopt(skill_constraints, q_current, kinematics);
//    robotTargetConfiguration = kin_solver_nlopt.solve();
    return true;
}


bool ConstraintSkill::solveDualQNLOpt(Eigen::VectorXd &robotTargetConfiguration)
{
    Eigen::VectorXd q1_current = q_current.segment(0,kinematic1->getDof());
    Eigen::VectorXd q2_current = q_current.segment(kinematic1->getDof(),kinematic2->getDof());
    DualArmKinematicSolverNLOpt solver_nlopt(skill_constraints, skill_constraints_left, skill_constraints_right, q1_current, q2_current, kinematic1, kinematic2);
    solver_nlopt.setSolverParams(this->solver_params);
    robotTargetConfiguration = solver_nlopt.solve();
    return true;
}

void ConstraintSkill::setSolverParams(SolverParams &params)
{
    this->solver_params = params;
}
