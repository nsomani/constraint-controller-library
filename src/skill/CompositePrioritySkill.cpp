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


#include "CompositePrioritySkill.h"

CompositePrioritySkill::CompositePrioritySkill():
    log(false)
{
}

CompositePrioritySkill::CompositePrioritySkill(rl::mdl::Kinematic *&kinematic, ObjectModel *&robot):
    CompositeSkill(kinematic, robot), log(false)
{

}

bool CompositePrioritySkill::solveQNLOpt(Eigen::VectorXd &robotTargetConfiguration)
{
    solveQNLOpt(robotTargetConfiguration, "COBYLA");
}

bool CompositePrioritySkill::solveQNLOpt(Eigen::VectorXd &robotTargetConfiguration, std::string solver_name)
{
    for(std::size_t priority_level = 1;priority_level <= skill_constraints.size();++priority_level)
    {
        bool low_priority_constraint_found = false;
        for(std::size_t constraint_id = 0;constraint_id < skill_constraints.size();++constraint_id)
        {
            if(skill_constraints[constraint_id]->priority_level > priority_level)
            {
                skill_constraints[constraint_id]->low_priority_constraint = true;
                skill_constraints[constraint_id]->high_priority_constraint = false;
//                std::cout << skill_constraints[constraint_id]->constraint_name << " low priority" << std::endl;
                low_priority_constraint_found = true;
            }
            else
            {
                skill_constraints[constraint_id]->low_priority_constraint = false;
//                std::cout << skill_constraints[constraint_id]->constraint_name << " not low priority" << std::endl;
            }
        }
        SolverNLOptGenericKinematic solver_nlopt(skill_constraints, q_current, kinematics, solver_name);
        solver_nlopt.setSolverParams(this->solver_params);        
        robotTargetConfiguration = solver_nlopt.solve();
//        bool result = false;
//        while(!result)
//        {
//            robotTargetConfiguration = solver_nlopt.solveResult(result);
//            std::cout << "solver failed, trying again" << std::endl;
//        }
        if(log)
        {
            constraintErrorVec = solver_nlopt.getConstraintErrors(robotTargetConfiguration);
        }
//        if(!solver.computeQd(skill_constraints, robotTargetConfiguration, kinematics, solver_name, num_iterations))
//            return false;
        if(!low_priority_constraint_found)
            return true;
        for(std::size_t constraint_id_post = 0;constraint_id_post < skill_constraints.size();++constraint_id_post)
        {
            if(skill_constraints[constraint_id_post]->minimization_constraint && !skill_constraints[constraint_id_post]->low_priority_constraint)
            {
//                std::cout << skill_constraints[constraint_id_post]->constraint_name << " high priority" << std::endl;
                skill_constraints[constraint_id_post]->high_priority_constraint = true;
                skill_constraints[constraint_id_post]->optimization_bound = skill_constraints[constraint_id_post]->calculateConstraintValue(robotTargetConfiguration)(0);
            }
        }
        std::cout << "level: " << priority_level << std::endl;
    }
    return true;
}

bool CompositePrioritySkill::solveDualQNLOpt(Eigen::VectorXd &robotTargetConfiguration)
{
    for(std::size_t priority_level = 1;priority_level <= skill_constraints.size();++priority_level)
    {
        bool low_priority_constraint_found = false;
        for(std::size_t constraint_id = 0;constraint_id < skill_constraints.size();++constraint_id)
        {
            if(skill_constraints[constraint_id]->priority_level > priority_level)
            {
                skill_constraints[constraint_id]->low_priority_constraint = true;
                skill_constraints[constraint_id]->high_priority_constraint = false;
//                std::cout << skill_constraints[constraint_id]->constraint_name << " low priority" << std::endl;
                low_priority_constraint_found = true;
            }
            else
            {
                skill_constraints[constraint_id]->low_priority_constraint = false;
//                std::cout << skill_constraints[constraint_id]->constraint_name << " not low priority" << std::endl;
            }
        }
        Eigen::VectorXd q1_current = q_current.segment(0,kinematic1->getDof());
        Eigen::VectorXd q2_current = q_current.segment(kinematic1->getDof(),kinematic2->getDof());
        DualArmKinematicSolverNLOpt solver_nlopt(skill_constraints, skill_constraints_left, skill_constraints_right, q1_current, q2_current, kinematic1, kinematic2);
        solver_nlopt.setSolverParams(this->solver_params);
        robotTargetConfiguration = solver_nlopt.solve();
//        if(!solver.computeQd(skill_constraints, robotTargetConfiguration, kinematics, solver_name, num_iterations))
//            return false;
//        if(!low_priority_constraint_found)
//            return true;
        for(std::size_t constraint_id_post = 0;constraint_id_post < skill_constraints.size();++constraint_id_post)
        {
            if(skill_constraints[constraint_id_post]->minimization_constraint && !skill_constraints[constraint_id_post]->low_priority_constraint)
            {
//                std::cout << skill_constraints[constraint_id_post]->constraint_name << " high priority" << std::endl;
                skill_constraints[constraint_id_post]->high_priority_constraint = true;
                skill_constraints[constraint_id_post]->optimization_bound = skill_constraints[constraint_id_post]->calculateConstraintValue(robotTargetConfiguration)(0);
            }
        }
//        std::cout << "level: " << priority_level << std::endl;
    }
    return true;
}

bool CompositePrioritySkill::solveTwoArmQNLOpt(Eigen::VectorXd &robotTargetConfiguration)
{
    Eigen::VectorXd q1_current = q_current.segment(0,kinematic1->getDof());
    Eigen::VectorXd q2_current = q_current.segment(kinematic1->getDof(),kinematic2->getDof());
    DualArmKinematicSolverNLOpt solver_nlopt(skill_constraints, skill_constraints_left, skill_constraints_right, q1_current, q2_current, kinematic1, kinematic2);
    solver_nlopt.setSolverParams(this->solver_params);
    robotTargetConfiguration = solver_nlopt.solve();
    return true;
}
