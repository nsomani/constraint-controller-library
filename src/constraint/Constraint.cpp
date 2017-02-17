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


#include "Constraint.h"

Constraint::Constraint(Space space, Level level, std::string name, bool minimization, int priority):
    constraint_space(space), constraint_level(level), constraint_name(name), minimization_constraint(minimization), priority_level(priority), low_priority_constraint(false), high_priority_constraint(false), constrained_shape_id(0), fixed_shape_id(-1)
{

}

bool Constraint::isConstraintSatisfied(Eigen::VectorXd &vec)
{
    Eigen::VectorXd val = calculateConstraintValue(vec);
    Eigen::VectorXd ub = getUpperBounds();
    Eigen::VectorXd lb = getLowerBounds();
    for(std::size_t constraint_id = 0;constraint_id < val.size();++constraint_id)
    {
        if(floatEqual(val(constraint_id), ub(constraint_id)) || floatEqual(val(constraint_id), lb(constraint_id)))
            continue;
        if(floatGreaterThan(val(constraint_id), ub(constraint_id)) || floatLessThan(val(constraint_id), lb(constraint_id)))
        {
//            std::cout << this->constraint_name << " NOT satisfied" << std::endl;
            return false;
        }
    }
//    std::cout << this->constraint_name << " satisfied" << std::endl;
    return true;
}

bool OperationalPositionConstraint::isConstraintSatisfied(Eigen::VectorXd &vec, bool use_quaternions)
{
    Eigen::VectorXd val;
    if(use_quaternions)
        val = calculateConstraintValue(vec);
    else
        val = calculateConstraintValueAA(vec);
    Eigen::VectorXd ub = getUpperBounds();
    Eigen::VectorXd lb = getLowerBounds();
    for(std::size_t constraint_id = 0;constraint_id < val.size();++constraint_id)
    {
        if(floatEqual(val(constraint_id), ub(constraint_id)) || floatEqual(val(constraint_id), lb(constraint_id)))
            continue;
        if(floatGreaterThan(val(constraint_id), ub(constraint_id)) || floatLessThan(val(constraint_id), lb(constraint_id)))
        {
//            std::cout << this->constraint_name << " NOT satisfied" << std::endl;
            return false;
        }
    }
//    std::cout << this->constraint_name << " satisfied" << std::endl;
    return true;
}

JointPositionConstraint::JointPositionConstraint(std::string name):
    Constraint(Constraint::Configuration, Constraint::Position, name)
{

}

JointVelocityConstraint::JointVelocityConstraint(std::string name):
    Constraint(Constraint::Configuration, Constraint::Velocity, name)
{

}

JointAccelerationConstraint::JointAccelerationConstraint(std::string name):
    Constraint(Constraint::Configuration, Constraint::Acceleration, name)
{

}

OperationalPositionConstraint::OperationalPositionConstraint(std::string name):
    Constraint(Constraint::Operational, Constraint::Position, name)
{

}

OperationalVelocityConstraint::OperationalVelocityConstraint(std::string name):
    Constraint(Constraint::Operational, Constraint::Velocity, name)
{

}

OperationalAccelerationConstraint::OperationalAccelerationConstraint(std::string name):
    Constraint(Constraint::Operational, Constraint::Acceleration, name)
{

}
