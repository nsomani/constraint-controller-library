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


#include "SolverNLOptGenericKinematic.h"

double SolverNLOptGenericKinematic::optimization_function_configuration_prioritized(const std::vector<double> &q, std::vector<double> &grad, void *data)
{
    SolverNLOptGenericKinematic* solver = (SolverNLOptGenericKinematic*) data;
    solver->num_f_calls_++;
    Eigen::VectorXd q_iter = Eigen::VectorXd::Map(q.data(), q.size());
    Eigen::VectorXd X_iter;
    if(solver->use_quaternions)
        X_iter = solver->solver_utils_->getTransformVectorFromQ(q_iter, solver->T_current_);
    else
        X_iter = solver->solver_utils_->getTransformVectorFromQAA(q_iter, solver->T_current_);

    bool min_constraint_found = false;
    double obj_value = 0;

    if (!grad.empty())
    {
        for (std::size_t i = 0; i < q.size(); ++i)
        {
            grad[i] = 0;
        }
    }

    for(std::size_t constraint_id = 0;constraint_id < solver->constraints_.size();++constraint_id)
    {
        if(solver->constraints_[constraint_id]->minimization_constraint && !solver->constraints_[constraint_id]->low_priority_constraint && !solver->constraints_[constraint_id]->high_priority_constraint)
        {
            min_constraint_found = true;
            Eigen::VectorXd g_constraint;
            bool satisfied = false;
            if(solver->constraints_[constraint_id]->constraint_space == Constraint::Configuration)
            {
                g_constraint = solver->constraints_[constraint_id]->calculateConstraintValue(q_iter);
                satisfied = solver->constraints_[constraint_id]->isConstraintSatisfied(q_iter);
            }
            else if(solver->constraints_[constraint_id]->constraint_space == Constraint::Operational && solver->constraints_[constraint_id]->constraint_level == Constraint::Position)
            {
                if(solver->use_quaternions)
                    g_constraint = solver->constraints_[constraint_id]->calculateConstraintValue(X_iter);
                else
                    g_constraint = ((OperationalPositionConstraint*)solver->constraints_[constraint_id])->calculateConstraintValueAA(X_iter);
                satisfied = ((OperationalPositionConstraint*)solver->constraints_[constraint_id])->isConstraintSatisfied(X_iter, solver->use_quaternions);
            }
            if(!satisfied)
                obj_value += g_constraint(0);

            if (!grad.empty())
            {
                Eigen::MatrixXd g_constraint_derivative;

                if(solver->constraints_[constraint_id]->constraint_space == Constraint::Configuration)
                {
    #ifdef NUMERICAL_DIFF
                      g_constraint_derivative = solver->solver_utils_->computeQConstraintDerivative(q_iter, solver->constraints_[constraint_id]);
    #endif
                }
                else  if(solver->constraints_[constraint_id]->constraint_space == Constraint::Operational && solver->constraints_[constraint_id]->constraint_level == Constraint::Position)
                {
                    OperationalPositionConstraint* constraint_operational = ((OperationalPositionConstraint*)solver->constraints_[constraint_id]);
    #ifdef NUMERICAL_DIFF
                    if(solver->use_quaternions)
                        g_constraint_derivative = solver->solver_utils_->computeXConstraintDerivative(q_iter, solver->constraints_[constraint_id], solver->T_current_);
                    else
                        g_constraint_derivative = solver->solver_utils_->computeXConstraintDerivativeAA(q_iter, constraint_operational, solver->T_current_);
    #endif
                }
                for (std::size_t i = 0; i < q.size(); ++i)
                {
                    grad[i] += g_constraint_derivative(i);
                }
            }
        }
    }
    if(!min_constraint_found)
    {
        for(std::size_t constraint_id = 0;constraint_id < solver->constraints_.size();++constraint_id)
        {
            if(solver->constraints_[constraint_id]->low_priority_constraint)
            {
                Eigen::VectorXd g_constraint;
                Eigen::MatrixXd g_constraint_derivative;
                bool satisfied = false;
                min_constraint_found = true;
                Eigen::VectorXd lb;
                Eigen::VectorXd ub;
//                lb = solver->constraints_[constraint_id]->getLowerBounds();
//                ub = solver->constraints_[constraint_id]->getUpperBounds();
                lb = solver->lb_[constraint_id];
                ub = solver->ub_[constraint_id];
                if(solver->constraints_[constraint_id]->constraint_space == Constraint::Configuration)
                {
                    g_constraint = solver->constraints_[constraint_id]->calculateConstraintValue(q_iter);
                    satisfied = solver->constraints_[constraint_id]->isConstraintSatisfied(q_iter);
                }
                else if(solver->constraints_[constraint_id]->constraint_space == Constraint::Operational && solver->constraints_[constraint_id]->constraint_level == Constraint::Position)
                {
                    if(solver->use_quaternions)
                        g_constraint = solver->constraints_[constraint_id]->calculateConstraintValue(X_iter);
                    else
                        g_constraint = ((OperationalPositionConstraint*)solver->constraints_[constraint_id])->calculateConstraintValueAA(X_iter);
                    satisfied = ((OperationalPositionConstraint*)solver->constraints_[constraint_id])->isConstraintSatisfied(X_iter, solver->use_quaternions);
                }
                if (!grad.empty())
                {
                    if(solver->constraints_[constraint_id]->constraint_space == Constraint::Configuration)
                    {
        #ifdef NUMERICAL_DIFF
                          g_constraint_derivative = solver->solver_utils_->computeQConstraintDerivative(q_iter, solver->constraints_[constraint_id]);
        #endif
                    }
                    else if(solver->constraints_[constraint_id]->constraint_space == Constraint::Operational && solver->constraints_[constraint_id]->constraint_level == Constraint::Position)
                    {
                        OperationalPositionConstraint* constraint_operational = ((OperationalPositionConstraint*)solver->constraints_[constraint_id]);
        #ifdef NUMERICAL_DIFF
                        if(solver->use_quaternions)
                            g_constraint_derivative = solver->solver_utils_->computeXConstraintDerivative(q_iter, solver->constraints_[constraint_id], solver->T_current_);
                        else
                            g_constraint_derivative = solver->solver_utils_->computeXConstraintDerivativeAA(q_iter, constraint_operational, solver->T_current_);
        #endif
                    }
                }

                if(!satisfied)
                {
                    if(solver->constraints_[constraint_id]->minimization_constraint)
                    {
                        obj_value += g_constraint(0);
                        if (!grad.empty())
                        {
                            for (std::size_t i = 0; i < q.size(); ++i)
                            {
                                grad[i] += g_constraint_derivative(i);
                            }
                        }
                    }
                    else
                    {
                        for(std::size_t dim = 0;dim < g_constraint.size();dim++)
                        {
                            if(floatGreaterThan(lb[dim], g_constraint[dim]))
                            {
                                obj_value += (lb[dim]-g_constraint[dim])*(lb[dim]-g_constraint[dim]);
                                if (!grad.empty())
                                {
                                    for (std::size_t i = 0; i < q.size(); ++i)
                                    {
                                        grad[i] += -2*g_constraint_derivative(dim, i)*(lb[dim]-g_constraint[dim]);
                                    }
                                }
                            }
                            else if(floatGreaterThan(g_constraint[dim], ub[dim]))
                            {
                                obj_value += (g_constraint[dim]-ub[dim])*(g_constraint[dim]-ub[dim]);
                                if (!grad.empty())
                                {
                                    for (std::size_t i = 0; i < q.size(); ++i)
                                    {
                                        grad[i] += 2*g_constraint_derivative(dim, i)*(g_constraint[dim]-ub[dim]);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    if(!min_constraint_found)
    {
        obj_value += (q_iter-solver->q_current_).squaredNorm();
        if (!grad.empty())
        {
            for (std::size_t i = 0; i < q.size(); ++i)
            {
                grad[i] = 2*(q[i]-solver->q_current_(i));
            }
        }
    }

    return obj_value;
}

void SolverNLOptGenericKinematic::nonlinear_inequality_constraints_configuration_prioritized(uint m, double* result, uint n, const double* x, double *grad, void* data)
{
    SolverNLOptGenericKinematic* solver = (SolverNLOptGenericKinematic*) data;
    solver->num_g_calls_++;
    Eigen::VectorXd q_iter = Eigen::VectorXd::Map(x, n);
    Eigen::VectorXd X_iter;
    if(solver->use_quaternions)
        X_iter = solver->solver_utils_->getTransformVectorFromQ(q_iter, solver->T_current_);
    else
        X_iter = solver->solver_utils_->getTransformVectorFromQAA(q_iter, solver->T_current_);

    std::size_t constraint_dim = 0;
    for(std::size_t constraint_id = 0;constraint_id < solver->constraints_.size();++constraint_id)
    {
        if(solver->constraints_[constraint_id]->minimization_constraint && !solver->constraints_[constraint_id]->high_priority_constraint)
            continue;
        if(solver->constraints_[constraint_id]->low_priority_constraint)
            continue;
        Eigen::VectorXd g_constraint;
        Eigen::MatrixXd g_constraint_derivative;
        Eigen::VectorXd lb;
        Eigen::VectorXd ub;
        if(solver->constraints_[constraint_id]->high_priority_constraint)
        {
            lb.resize(1);ub.resize(1);
            lb(0) = solver->constraints_[constraint_id]->optimization_bound-1e-10;
            ub(0) = solver->constraints_[constraint_id]->optimization_bound+1e-10;
        }
        else
        {
//            lb = solver->constraints_[constraint_id]->getLowerBounds();
//            ub = solver->constraints_[constraint_id]->getUpperBounds();
            lb = solver->lb_[constraint_id];
            ub = solver->ub_[constraint_id];
        }

        if(solver->constraints_[constraint_id]->constraint_space == Constraint::Configuration)
        {
            g_constraint = solver->constraints_[constraint_id]->calculateConstraintValue(q_iter);
        }
        else if(solver->constraints_[constraint_id]->constraint_space == Constraint::Operational && solver->constraints_[constraint_id]->constraint_level == Constraint::Position)
        {
            if(solver->use_quaternions)
                g_constraint = solver->constraints_[constraint_id]->calculateConstraintValue(X_iter);
            else
                g_constraint = ((OperationalPositionConstraint*)solver->constraints_[constraint_id])->calculateConstraintValueAA(X_iter);
        }
        if (grad != NULL)
        {
            if(solver->constraints_[constraint_id]->constraint_space == Constraint::Configuration)
            {
#ifdef NUMERICAL_DIFF
                  g_constraint_derivative = solver->solver_utils_->computeQConstraintDerivative(q_iter, solver->constraints_[constraint_id]);
//                g_constraint_derivative = solver->constraints_[constraint_id]->calculateConstraintDerivative(q_iter);
#endif
            }
            else if(solver->constraints_[constraint_id]->constraint_space == Constraint::Operational && solver->constraints_[constraint_id]->constraint_level == Constraint::Position)
            {
                OperationalPositionConstraint* constraint_operational = ((OperationalPositionConstraint*)solver->constraints_[constraint_id]);
#ifdef NUMERICAL_DIFF
                if(solver->use_quaternions)
                    g_constraint_derivative = solver->solver_utils_->computeXConstraintDerivative(q_iter, solver->constraints_[constraint_id], solver->T_current_);
                else
                    g_constraint_derivative = solver->solver_utils_->computeXConstraintDerivativeAA(q_iter, constraint_operational, solver->T_current_);
#endif
            }
        }
        if(solver->constraints_[constraint_id]->high_priority_constraint)
        {
            result[2*constraint_dim] = g_constraint.norm()-ub(0);
            result[2*constraint_dim+1] = -1*g_constraint.norm()+lb(0);
            if (grad != NULL)
            {
                for(std::size_t variable_id = 0;variable_id < solver->num_variables_;++variable_id)
                {
                    grad[(2*constraint_dim)*solver->num_variables_+variable_id] = g_constraint_derivative.col(variable_id).norm();
                    grad[(2*constraint_dim+1)*solver->num_variables_+variable_id] = -1*g_constraint_derivative.col(variable_id).norm();
                }
            }
            constraint_dim++;
        }
        else
        {
            for(std::size_t constraint_i_dim = 0;constraint_i_dim < g_constraint.size();++constraint_i_dim, constraint_dim++)
            {
                result[2*constraint_dim] = g_constraint(constraint_i_dim)-ub(constraint_i_dim);
                result[2*constraint_dim+1] = -1*g_constraint(constraint_i_dim)+lb(constraint_i_dim);
                if (grad != NULL)
                {
                    for(std::size_t variable_id = 0;variable_id < solver->num_variables_;++variable_id)
                    {
                        grad[(2*constraint_dim)*solver->num_variables_+variable_id] = g_constraint_derivative(constraint_i_dim, variable_id);
                        grad[(2*constraint_dim+1)*solver->num_variables_+variable_id] = -1*g_constraint_derivative(constraint_i_dim, variable_id);
                    }
                }
            }
        }
    }
}

SolverNLOptGenericKinematic::SolverNLOptGenericKinematic(std::vector<Constraint *> &constraints, Eigen::VectorXd q_current, rl::mdl::Kinematic *kinematics, std::string solver_name):
    constraints_(constraints), q_current_(q_current), kinematics_(kinematics), use_quaternions(true)
{
    kinematics_->setPosition(q_current_);
    kinematics_->forwardPosition();
    X_current_.resize(7*kinematics_->getOperationalDof());
    for(std::size_t ef_id = 0;ef_id < kinematics_->getOperationalDof();++ef_id)
    {
        T_current_.push_back(kinematics_->getOperationalPosition(ef_id).matrix());
        X_current_.segment(7*ef_id,7) = vectorFromTransform(T_current_.back());
    }
    num_variables_ = q_current.size();
    if(solver_name == "COBYLA")
        opt_ = new nlopt::opt(nlopt::LN_COBYLA, num_variables_);
    if(solver_name == "BOBYQA")
        opt_ = new nlopt::opt(nlopt::LN_BOBYQA, num_variables_);
    else if(solver_name == "SLSQP")
        opt_ = new nlopt::opt(nlopt::LD_SLSQP, num_variables_);
    else if(solver_name == "MMA")
        opt_ = new nlopt::opt(nlopt::LD_MMA, num_variables_);
    else
        opt_ = new nlopt::opt(nlopt::LN_COBYLA, num_variables_);
    solver_utils_ = new SolverUtils(kinematics_);
}

SolverNLOptGenericKinematic::SolverNLOptGenericKinematic(std::vector<Constraint *> &constraints, Eigen::VectorXd q_current, rl::mdl::Kinematic *kinematics):
    SolverNLOptGenericKinematic(constraints, q_current, kinematics, "COBYLA")
{
}

Eigen::VectorXd SolverNLOptGenericKinematic::solveResult(bool &result)
{
    std::vector<double> tolerances;
    for(std::size_t constraint_id = 0;constraint_id < constraints_.size();++constraint_id)
    {
        lb_.push_back(constraints_[constraint_id]->getLowerBounds());
        ub_.push_back(constraints_[constraint_id]->getUpperBounds());
        if(constraints_[constraint_id]->minimization_constraint && !constraints_[constraint_id]->high_priority_constraint)
            continue;
        if(constraints_[constraint_id]->low_priority_constraint)
            continue;
        for(std::size_t constraint_i_dim = 0;constraint_i_dim < constraints_[constraint_id]->getNumConstraints();++constraint_i_dim)
        {
            tolerances.push_back(solver_params_.constraint_tol_);
            tolerances.push_back(solver_params_.constraint_tol_);
        }
    }
    Eigen::VectorXd lb, ub;
    lb.resize(kinematics_->getDof());
    ub.resize(kinematics_->getDof());
    kinematics_->getMinimum(lb);
    kinematics_->getMaximum(ub);
    std::vector<double> ub_vec, lb_vec;
    for(std::size_t var_id = 0;var_id < ub.size();++var_id)
    {
        lb_vec.push_back(lb(var_id));
        ub_vec.push_back(ub(var_id));
    }

    opt_->set_min_objective(SolverNLOptGenericKinematic::optimization_function_configuration_prioritized, this);
    opt_->set_stopval(solver_params_.stopval_);
    opt_->set_ftol_abs(solver_params_.f_tol_);
    opt_->set_xtol_abs(solver_params_.x_tol_);

    opt_->set_lower_bounds(lb_vec);
    opt_->set_upper_bounds(ub_vec);

    opt_->set_maxtime(solver_params_.max_time_);
    opt_->add_inequality_mconstraint(SolverNLOptGenericKinematic::nonlinear_inequality_constraints_configuration_prioritized, this, tolerances);
    std::vector<double> q;
    q.resize(num_variables_);
    for(std::size_t variable_id = 0;variable_id < num_variables_;++variable_id)
        q[variable_id] = q_current_(variable_id);
    double f_val = 0;
    try
    {
        nlopt::result res = opt_->optimize(q, f_val);
//        std::cout << "optimization result:" << res << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }
    Eigen::VectorXd optimal_value = Eigen::VectorXd::Map(q.data(), q.size());
//    std::cout << num_f_calls_ << ", " << num_g_calls_ << std::endl;
//    std::cout << "optimal value:" << optimal_value.transpose() << " f_val: " << f_val << std::endl;
//    Eigen::VectorXd q_iter = optimal_value;
//    Eigen::VectorXd X_iter = solver_utils_->getTransformVectorFromQ(q_iter,T_current_);
//    for(std::size_t constraint_id = 0;constraint_id < constraints_.size();++constraint_id)
//    {
//        bool satisfied = false;
//        Eigen::VectorXd g_constraint;
//        Eigen::VectorXd lb = constraints_[constraint_id]->getLowerBounds();
//        Eigen::VectorXd ub = constraints_[constraint_id]->getUpperBounds();
//        if(constraints_[constraint_id]->constraint_space == Constraint::Configuration)
//        {
//            g_constraint = constraints_[constraint_id]->calculateConstraintValue(q_iter);
//            satisfied = constraints_[constraint_id]->isConstraintSatisfied(q_iter);
//        }
//        else
//        {
//            g_constraint = constraints_[constraint_id]->calculateConstraintValue(X_iter);
//            satisfied = constraints_[constraint_id]->isConstraintSatisfied(X_iter);
//        }
//        if(!satisfied)
//            result = false;
//        if(!constraints_[constraint_id]->minimization_constraint)
//            std::cout << constraints_[constraint_id]->constraint_name << ":" << lb.transpose() << " < " << g_constraint.transpose() << " < " << ub.transpose() << std::endl;
//        else if (constraints_[constraint_id]->high_priority_constraint)
//            std::cout << constraints_[constraint_id]->constraint_name << ":" << constraints_[constraint_id]->optimization_bound << " < " << g_constraint.transpose() << " < " << constraints_[constraint_id]->optimization_bound << std::endl;
//        else
//            std::cout << constraints_[constraint_id]->constraint_name << ":" << g_constraint.transpose() << std::endl;
//    }
    return optimal_value;
}

Eigen::VectorXd SolverNLOptGenericKinematic::solve()
{
    bool result = true;
    return solveResult(result);
}


Eigen::VectorXd SolverNLOptGenericKinematic::getConstraintErrors(Eigen::VectorXd optimal_value)
{
    Eigen::VectorXd constraintErrors;
    constraintErrors.resize(constraints_.size());
    Eigen::VectorXd q_iter = optimal_value;
    Eigen::VectorXd X_iter = solver_utils_->getTransformVectorFromQ(q_iter, T_current_);
    for(std::size_t constraint_id = 0;constraint_id < constraints_.size();++constraint_id)
    {
        Eigen::VectorXd g_constraint;
        Eigen::VectorXd lb = constraints_[constraint_id]->getLowerBounds();
        Eigen::VectorXd ub = constraints_[constraint_id]->getUpperBounds();
        bool satisfied = false;
        if(constraints_[constraint_id]->constraint_space == Constraint::Configuration)
        {
            g_constraint = constraints_[constraint_id]->calculateConstraintValue(q_iter);
            satisfied = constraints_[constraint_id]->isConstraintSatisfied(q_iter);
        }
        else
        {
            g_constraint = constraints_[constraint_id]->calculateConstraintValue(X_iter);
            satisfied = constraints_[constraint_id]->isConstraintSatisfied(X_iter);
        }
        if(!constraints_[constraint_id]->minimization_constraint)
        {
            if(satisfied)
                constraintErrors(constraint_id) = 0;
            else
            {   constraintErrors(constraint_id) = 0;
                for(std::size_t dim = 0;dim < g_constraint.size();++dim)
                {
                    if(floatEqual(g_constraint(dim), lb(dim)) || floatEqual(g_constraint(dim), ub(dim)))
                        continue;
                    if(floatLessThan(g_constraint(dim), lb(dim)))
                        constraintErrors(constraint_id) += (g_constraint(dim)-lb(dim))*(g_constraint(dim)-lb(dim));
                    if(floatGreaterThan(g_constraint(dim), ub(dim)))
                        constraintErrors(constraint_id) += (ub(dim)-g_constraint(dim))*(ub(dim)-g_constraint(dim));
                }
                constraintErrors(constraint_id) = std::sqrt(constraintErrors(constraint_id));
//                constraintErrors(constraint_id) = std::min((g_constraint-ub).norm(),(g_constraint-lb).norm());
            }
        }
        else if (constraints_[constraint_id]->high_priority_constraint)
        {
            if(satisfied && floatLessThan(g_constraint.norm(), constraints_[constraint_id]->optimization_bound))
                constraintErrors(constraint_id) = 0;
            else
            {
                constraintErrors(constraint_id) = std::fabs(constraints_[constraint_id]->optimization_bound-g_constraint.norm());
            }
        }
        else
        {
            constraintErrors(constraint_id) = g_constraint.norm();
        }
    }
    return constraintErrors;
}
