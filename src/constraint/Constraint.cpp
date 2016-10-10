

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
