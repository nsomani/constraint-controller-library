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


#include "SolverNLOpt.h"

SolverParams::SolverParams():
    f_tol_(1e-8), x_tol_(1e-8), constraint_tol_(1e-7), max_time_(1e-3), stopval_(1e-8)
{

}

SolverNLOpt::SolverNLOpt()
{
    num_f_calls_ = 0;
    num_g_calls_ = 0;
}

void SolverNLOpt::setSolverParams(SolverParams &params)

{
    solver_params_ = params;
}


GeometricSolverNLOpt::GeometricSolverNLOpt(std::vector<Constraint *> &constraints, Eigen::VectorXd &X_current):
    constraints_(constraints), X_current_(X_current)
{
    num_variables_ = X_current.size();
    opt_ = new nlopt::opt(nlopt::LN_COBYLA, num_variables_);
}

double GeometricSolverNLOpt::optimization_function_operational(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    GeometricSolverNLOpt* solver = (GeometricSolverNLOpt*) data;
    Eigen::VectorXd X_tr = Eigen::VectorXd::Map(x.data(), x.size());
    Eigen::VectorXd X_iter = applyTransform(X_tr, solver->X_current_);

//    if (!grad.empty()) {
//      for (std::size_t i=0; i<x.size(); ++i) {
//        grad[i]=2*(x[i]-solver->X_current_(i));
//      }
//    }
//    return (X_iter-solver->X_current_).squaredNorm();


//    double distance_rotation = distanceRotations(X_iter.segment(3,4), solver->X_current_.segment(3,4));
//    Eigen::VectorXd val;
//    val.resize(1);
//    val(0) = (X_iter.segment(0,3)-solver->X_current_.segment(0,3)).transpose()*(X_iter.segment(0,3)-solver->X_current_.segment(0,3));
//    val(0) += distance_rotation*distance_rotation;

    Eigen::VectorXd val;
    val.resize(1);
    val(0) += distanceAASS(X_iter, solver->X_current_);

    if (!grad.empty())
    {
        Eigen::VectorXd dX_minus, dX_plus;
        Eigen::VectorXd X_minus, X_plus;
        dX_minus.resize(7);
        dX_plus.resize(7);
        double eps = 1e-8;
        for(std::size_t dx_id = 0;dx_id < 7;dx_id++)
        {
            dX_minus = X_tr;
            dX_plus = X_tr;
            dX_minus(dx_id) -= eps;
            dX_plus(dx_id) += eps;
            dX_minus.segment(3,3).normalize();
            dX_plus.segment(3,3).normalize();
            X_minus = applyTransform(X_minus, solver->X_current_);
            X_plus = applyTransform(X_plus, solver->X_current_);
//            double val_minus = distanceRotations(X_minus.segment(3,4), solver->X_current_.segment(3,4));
//            val_minus *= val_minus;
//            val_minus += (X_minus.segment(0,3)-solver->X_current_.segment(0,3)).transpose()*(X_minus.segment(0,3)-solver->X_current_.segment(0,3));
//            double val_plus = distanceRotations(X_plus.segment(3,4), solver->X_current_.segment(3,4));
//            val_plus *= val_plus;
//            val_plus += (X_plus.segment(0,3)-solver->X_current_.segment(0,3)).transpose()*(X_plus.segment(0,3)-solver->X_current_.segment(0,3));
            double val_minus = distanceAASS(X_minus, solver->X_current_);
            double val_plus = distanceAASS(X_plus, solver->X_current_);
            grad[dx_id] = (val_plus-val_minus)/(2.0*eps);
        }
    }

    return val(0);
}

double KinematicSolverNLOpt::optimization_function_configuration(const std::vector<double> &q, std::vector<double> &grad, void *data)
{
    KinematicSolverNLOpt* solver = (KinematicSolverNLOpt*) data;
    solver->num_f_calls_++;
    Eigen::VectorXd q_iter = Eigen::VectorXd::Map(q.data(), q.size());
    if (!grad.empty()) {
      for (std::size_t i=0; i<q.size(); ++i) {
        grad[i]=2*(q[i]-solver->q_current_(i));
      }
    }
    return (q_iter-solver->q_current_).squaredNorm();
}

double PrioritizedKinematicSolverNLOpt::optimization_function_configuration_prioritized(const std::vector<double> &q, std::vector<double> &grad, void *data)
{
    PrioritizedKinematicSolverNLOpt* solver = (PrioritizedKinematicSolverNLOpt*) data;
    Eigen::VectorXd q_iter = Eigen::VectorXd::Map(q.data(), q.size());
    Eigen::VectorXd X_iter = solver->solver_utils_->getTransformVectorFromQ(q_iter, solver->T_current_);
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
            else
            {
                g_constraint = solver->constraints_[constraint_id]->calculateConstraintValue(X_iter);
                satisfied = solver->constraints_[constraint_id]->isConstraintSatisfied(X_iter);
            }
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
                else if(solver->constraints_[constraint_id]->constraint_space == Constraint::Operational)
                {
    #ifdef NUMERICAL_DIFF
                    g_constraint_derivative = solver->solver_utils_->computeXConstraintDerivative(q_iter, solver->constraints_[constraint_id], solver->T_current_);
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
                lb = solver->constraints_[constraint_id]->getLowerBounds();
                ub = solver->constraints_[constraint_id]->getUpperBounds();
                if(solver->constraints_[constraint_id]->constraint_space == Constraint::Configuration)
                {
                    g_constraint = solver->constraints_[constraint_id]->calculateConstraintValue(q_iter);
                    satisfied = solver->constraints_[constraint_id]->isConstraintSatisfied(q_iter);
                }
                else
                {
                    g_constraint = solver->constraints_[constraint_id]->calculateConstraintValue(X_iter);
                    satisfied = solver->constraints_[constraint_id]->isConstraintSatisfied(X_iter);
                }
                if (!grad.empty())
                {
                    if(solver->constraints_[constraint_id]->constraint_space == Constraint::Configuration)
                    {
        #ifdef NUMERICAL_DIFF
                          g_constraint_derivative = solver->solver_utils_->computeQConstraintDerivative(q_iter, solver->constraints_[constraint_id]);
        #endif
                    }
                    else if(solver->constraints_[constraint_id]->constraint_space == Constraint::Operational)
                    {
        #ifdef NUMERICAL_DIFF
                        g_constraint_derivative = solver->solver_utils_->computeXConstraintDerivative(q_iter, solver->constraints_[constraint_id], solver->T_current_);
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
                        for(std::size_t dim = 0;dim < g_constraint.size();++dim)
                        {
                            if(floatGreaterThan(lb[dim], g_constraint[dim]))
                            {
                                obj_value += (lb[dim]-g_constraint[dim])*(lb[dim]-g_constraint[dim]);
                                if (!grad.empty())
                                {
                                    grad[dim] += (lb[dim]-g_constraint[dim]);
                                }
                            }
                            else if(floatGreaterThan(g_constraint[dim], ub[dim]))
                            {
                                obj_value += (g_constraint[dim]-ub[dim])*(g_constraint[dim]-ub[dim]);
                                if (!grad.empty())
                                {
                                    grad[dim] += (g_constraint[dim]-ub[dim]);
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

double DualArmKinematicSolverNLOpt::optimization_function_dual_configuration_prioritized(const std::vector<double> &q, std::vector<double> &grad, void *data)
{
    DualArmKinematicSolverNLOpt* solver = (DualArmKinematicSolverNLOpt*) data;
    solver->num_f_calls_++;
    Eigen::VectorXd q_iter = Eigen::VectorXd::Map(q.data(), q.size());
    Eigen::VectorXd q1_iter = q_iter.segment(0,solver->kinematics1_->getDof());
    Eigen::VectorXd q2_iter = q_iter.segment(solver->kinematics1_->getDof(),solver->kinematics2_->getDof());
    Eigen::VectorXd X_iter = solver->solver_utils_->getTransformVectorFromDualQ(q1_iter, q2_iter, solver->T1_current_, solver->T2_current_);
    Eigen::VectorXd X1_iter = X_iter.segment(0,7);
    Eigen::VectorXd X2_iter = X_iter.segment(7,7);
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
            else
            {
                g_constraint = solver->constraints_[constraint_id]->calculateConstraintValue(X_iter);
                satisfied = solver->constraints_[constraint_id]->isConstraintSatisfied(X_iter);
            }
            obj_value += g_constraint(0);

            if (!grad.empty())
            {
                Eigen::MatrixXd g_constraint_derivative;

                if(solver->constraints_[constraint_id]->constraint_space == Constraint::Configuration)
                {
    #ifdef NUMERICAL_DIFF
                      g_constraint_derivative = solver->solver_utils_->computeDualQConstraintDerivative(q_iter, solver->constraints_[constraint_id]);
    #endif
                }
                else if(solver->constraints_[constraint_id]->constraint_space == Constraint::Operational)
                {
    #ifdef NUMERICAL_DIFF
                    g_constraint_derivative = solver->solver_utils_->computeDualXConstraintDerivative(q_iter, solver->constraints_[constraint_id], solver->T1_current_, solver->T2_current_);
    #endif
                }
                for (std::size_t i = 0; i < q.size(); ++i)
                {
                    grad[i] += g_constraint_derivative(i);
                }
            }
        }
    }
    for(std::size_t constraint_id = 0;constraint_id < solver->constraints_left_.size();++constraint_id)
    {
        if(solver->constraints_left_[constraint_id]->minimization_constraint && !solver->constraints_left_[constraint_id]->low_priority_constraint && !solver->constraints_left_[constraint_id]->high_priority_constraint)
        {
            min_constraint_found = true;
            Eigen::VectorXd g_constraint;
            bool satisfied = false;
            if(solver->constraints_left_[constraint_id]->constraint_space == Constraint::Configuration)
            {
                g_constraint = solver->constraints_left_[constraint_id]->calculateConstraintValue(q1_iter);
                satisfied = solver->constraints_left_[constraint_id]->isConstraintSatisfied(q1_iter);
            }
            else
            {
                g_constraint = solver->constraints_left_[constraint_id]->calculateConstraintValue(X1_iter);
                satisfied = solver->constraints_left_[constraint_id]->isConstraintSatisfied(X1_iter);
            }
            obj_value += g_constraint(0);

            if (!grad.empty())
            {
                Eigen::MatrixXd g_constraint_derivative;

                if(solver->constraints_left_[constraint_id]->constraint_space == Constraint::Configuration)
                {
    #ifdef NUMERICAL_DIFF
                    g_constraint_derivative = solver->solver_utils_->computeQConstraintDerivative(q1_iter, solver->constraints_left_[constraint_id]);
    #endif
                }
                else if(solver->constraints_left_[constraint_id]->constraint_space == Constraint::Operational)
                {
    #ifdef NUMERICAL_DIFF
                    g_constraint_derivative = solver->solver_utils_->computeXConstraintDerivative(q1_iter, solver->constraints_left_[constraint_id], solver->T1_current_, solver->kinematics1_);
    #endif
                }
                for (std::size_t i = 0; i < solver->kinematics1_->getDof(); ++i)
                {
                    grad[i] += g_constraint_derivative(i);
                }
            }
        }
    }
    for(std::size_t constraint_id = 0;constraint_id < solver->constraints_right_.size();++constraint_id)
    {
        if(solver->constraints_right_[constraint_id]->minimization_constraint && !solver->constraints_right_[constraint_id]->low_priority_constraint && !solver->constraints_right_[constraint_id]->high_priority_constraint)
        {
            min_constraint_found = true;
            Eigen::VectorXd g_constraint;
            bool satisfied = false;
            if(solver->constraints_right_[constraint_id]->constraint_space == Constraint::Configuration)
            {
                g_constraint = solver->constraints_right_[constraint_id]->calculateConstraintValue(q2_iter);
                satisfied = solver->constraints_right_[constraint_id]->isConstraintSatisfied(q2_iter);
            }
            else
            {
                g_constraint = solver->constraints_right_[constraint_id]->calculateConstraintValue(X2_iter);
                satisfied = solver->constraints_right_[constraint_id]->isConstraintSatisfied(X2_iter);
            }
            obj_value += g_constraint(0);

            if (!grad.empty())
            {
                Eigen::MatrixXd g_constraint_derivative;

                if(solver->constraints_right_[constraint_id]->constraint_space == Constraint::Configuration)
                {
    #ifdef NUMERICAL_DIFF
                    g_constraint_derivative = solver->solver_utils_->computeQConstraintDerivative(q2_iter, solver->constraints_right_[constraint_id]);
    #endif
                }
                else if(solver->constraints_right_[constraint_id]->constraint_space == Constraint::Operational)
                {
    #ifdef NUMERICAL_DIFF
                    g_constraint_derivative = solver->solver_utils_->computeXConstraintDerivative(q2_iter, solver->constraints_right_[constraint_id], solver->T2_current_, solver->kinematics2_);
    #endif
                }
                for (std::size_t i = 0; i < solver->kinematics2_->getDof(); ++i)
                {
                    grad[solver->kinematics1_->getDof()+i] += g_constraint_derivative(i);
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
                lb = solver->constraints_[constraint_id]->getLowerBounds();
                ub = solver->constraints_[constraint_id]->getUpperBounds();
                if(solver->constraints_[constraint_id]->constraint_space == Constraint::Configuration)
                {
                    g_constraint = solver->constraints_[constraint_id]->calculateConstraintValue(q_iter);
                    satisfied = solver->constraints_[constraint_id]->isConstraintSatisfied(q_iter);
                }
                else
                {
                    g_constraint = solver->constraints_[constraint_id]->calculateConstraintValue(X_iter);
                    satisfied = solver->constraints_[constraint_id]->isConstraintSatisfied(X_iter);
                }
                if (!grad.empty())
                {
                    if(solver->constraints_[constraint_id]->constraint_space == Constraint::Configuration)
                    {
        #ifdef NUMERICAL_DIFF
                          g_constraint_derivative = solver->solver_utils_->computeDualQConstraintDerivative(q_iter, solver->constraints_[constraint_id]);
        #endif
                    }
                    else if(solver->constraints_[constraint_id]->constraint_space == Constraint::Operational)
                    {
        #ifdef NUMERICAL_DIFF
                        g_constraint_derivative = solver->solver_utils_->computeDualXConstraintDerivative(q_iter, solver->constraints_[constraint_id], solver->T1_current_, solver->T2_current_);
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
                        for(std::size_t dim = 0;dim < g_constraint.size();++dim)
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
        for(std::size_t constraint_id = 0;constraint_id < solver->constraints_left_.size();++constraint_id)
        {
            if(solver->constraints_left_[constraint_id]->low_priority_constraint)
            {
                Eigen::VectorXd g_constraint;
                Eigen::MatrixXd g_constraint_derivative;
                bool satisfied = false;
                min_constraint_found = true;
                Eigen::VectorXd lb;
                Eigen::VectorXd ub;
                lb = solver->constraints_left_[constraint_id]->getLowerBounds();
                ub = solver->constraints_left_[constraint_id]->getUpperBounds();
                if(solver->constraints_left_[constraint_id]->constraint_space == Constraint::Configuration)
                {
                    g_constraint = solver->constraints_left_[constraint_id]->calculateConstraintValue(q1_iter);
                    satisfied = solver->constraints_left_[constraint_id]->isConstraintSatisfied(q1_iter);
                }
                else
                {
                    g_constraint = solver->constraints_left_[constraint_id]->calculateConstraintValue(X1_iter);
                    satisfied = solver->constraints_left_[constraint_id]->isConstraintSatisfied(X1_iter);
                }
                if (!grad.empty())
                {
                    if(solver->constraints_left_[constraint_id]->constraint_space == Constraint::Configuration)
                    {
        #ifdef NUMERICAL_DIFF
                          g_constraint_derivative = solver->solver_utils_->computeQConstraintDerivative(q1_iter, solver->constraints_left_[constraint_id]);
        #endif
                    }
                    else if(solver->constraints_left_[constraint_id]->constraint_space == Constraint::Operational)
                    {
        #ifdef NUMERICAL_DIFF
                        g_constraint_derivative = solver->solver_utils_->computeXConstraintDerivative(q1_iter, solver->constraints_left_[constraint_id], solver->T1_current_);
        #endif
                    }
                }

                if(!satisfied)
                {
                    if(solver->constraints_left_[constraint_id]->minimization_constraint)
                    {
                        obj_value += g_constraint(0);
                        if (!grad.empty())
                        {
                            for (std::size_t i = 0; i < solver->kinematics1_->getDof(); ++i)
                            {
                                grad[i] += g_constraint_derivative(i);
                            }
                        }
                    }
                    else
                    {
                        for(std::size_t dim = 0;dim < g_constraint.size();++dim)
                        {
                            if(floatGreaterThan(lb[dim], g_constraint[dim]))
                            {
                                obj_value += (lb[dim]-g_constraint[dim])*(lb[dim]-g_constraint[dim]);
                                if (!grad.empty())
                                {
                                    for (std::size_t i = 0; i < solver->kinematics1_->getDof(); ++i)
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
                                    for (std::size_t i = 0; i < solver->kinematics1_->getDof(); ++i)
                                    {
                                        grad[i] += 2*g_constraint_derivative(dim, i)*((g_constraint[dim]-ub[dim]));
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        for(std::size_t constraint_id = 0;constraint_id < solver->constraints_right_.size();++constraint_id)
        {
            if(solver->constraints_right_[constraint_id]->low_priority_constraint)
            {
                Eigen::VectorXd g_constraint;
                Eigen::MatrixXd g_constraint_derivative;
                bool satisfied = false;
                min_constraint_found = true;
                Eigen::VectorXd lb;
                Eigen::VectorXd ub;
                lb = solver->constraints_right_[constraint_id]->getLowerBounds();
                ub = solver->constraints_right_[constraint_id]->getUpperBounds();
                if(solver->constraints_right_[constraint_id]->constraint_space == Constraint::Configuration)
                {
                    g_constraint = solver->constraints_right_[constraint_id]->calculateConstraintValue(q2_iter);
                    satisfied = solver->constraints_right_[constraint_id]->isConstraintSatisfied(q2_iter);
                }
                else
                {
                    g_constraint = solver->constraints_right_[constraint_id]->calculateConstraintValue(X2_iter);
                    satisfied = solver->constraints_right_[constraint_id]->isConstraintSatisfied(X2_iter);
                }
                if (!grad.empty())
                {
                    if(solver->constraints_right_[constraint_id]->constraint_space == Constraint::Configuration)
                    {
        #ifdef NUMERICAL_DIFF
                          g_constraint_derivative = solver->solver_utils_->computeQConstraintDerivative(q2_iter, solver->constraints_right_[constraint_id]);
        #endif
                    }
                    else if(solver->constraints_right_[constraint_id]->constraint_space == Constraint::Operational)
                    {
        #ifdef NUMERICAL_DIFF
                        g_constraint_derivative = solver->solver_utils_->computeXConstraintDerivative(q2_iter, solver->constraints_right_[constraint_id], solver->T2_current_);
        #endif
                    }
                }

                if(!satisfied)
                {
                    if(solver->constraints_right_[constraint_id]->minimization_constraint)
                    {
                        obj_value += g_constraint(0);
                        if (!grad.empty())
                        {
                            for (std::size_t i = 0; i < solver->kinematics2_->getDof(); ++i)
                            {
                                grad[solver->kinematics1_->getDof()+i] += g_constraint_derivative(i);
                            }
                        }
                    }
                    else
                    {
                        for(std::size_t dim = 0;dim < g_constraint.size();++dim)
                        {
                            if(floatGreaterThan(lb[dim], g_constraint[dim]))
                            {
                                obj_value += (lb[dim]-g_constraint[dim])*(lb[dim]-g_constraint[dim]);
                                if (!grad.empty())
                                {
                                    for (std::size_t i = 0; i < solver->kinematics2_->getDof(); ++i)
                                    {
                                        grad[solver->kinematics1_->getDof()+i] += -2*g_constraint_derivative(dim, i)*(lb[dim]-g_constraint[dim]);
                                    }
                                }
                            }
                            else if(floatGreaterThan(g_constraint[dim], ub[dim]))
                            {
                                obj_value += (g_constraint[dim]-ub[dim])*(g_constraint[dim]-ub[dim]);
                                if (!grad.empty())
                                {
                                    for (std::size_t i = 0; i < solver->kinematics2_->getDof(); ++i)
                                    {
                                        grad[solver->kinematics1_->getDof()+i] += 2*g_constraint_derivative(dim, i)*(g_constraint[dim]-ub[dim]);
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
        obj_value += (q1_iter-solver->q1_current_).squaredNorm();
        obj_value += (q2_iter-solver->q2_current_).squaredNorm();
        if (!grad.empty())
        {
            for (std::size_t i = 0; i < q1_iter.size(); ++i)
            {
                grad[i] = 2*(q[i]-solver->q1_current_(i));
            }
            for (std::size_t i = 0; i < q2_iter.size(); ++i)
            {
                grad[q1_iter.size() + i] = 2*(q[q1_iter.size() + i]-solver->q2_current_(i));
            }
        }
    }

    return obj_value;
}

void GeometricSolverNLOpt::nonlinear_inequality_constraints_operational(uint m, double* result, uint n, const double* x, double *grad, void* data)
{
    GeometricSolverNLOpt* solver = (GeometricSolverNLOpt*) data;
    Eigen::VectorXd X_iter = Eigen::VectorXd::Map(x, n);
    std::size_t constraint_dim = 0;
    for(std::size_t constraint_id = 0;constraint_id < solver->constraints_.size();++constraint_id)
    {
        Eigen::VectorXd g_constraint;
        Eigen::MatrixXd g_constraint_derivative;
        Eigen::VectorXd lb;
        Eigen::VectorXd ub;
        if(solver->constraints_[constraint_id]->constraint_space == Constraint::Operational)
        {
            g_constraint = solver->constraints_[constraint_id]->calculateConstraintValue(X_iter);
            lb = solver->constraints_[constraint_id]->getLowerBounds();
            ub = solver->constraints_[constraint_id]->getUpperBounds();
        }
        if (grad != NULL)
        {
            if(solver->constraints_[constraint_id]->constraint_space == Constraint::Operational)
                g_constraint_derivative = solver->constraints_[constraint_id]->calculateConstraintDerivative(X_iter);
        }
        for(std::size_t constraint_i_dim = 0;constraint_i_dim < g_constraint.size();++constraint_i_dim, ++constraint_dim)
        {
            result[2*constraint_dim] = g_constraint[constraint_i_dim]-ub(constraint_i_dim);
            result[2*constraint_dim+1] = -1*g_constraint[constraint_i_dim]+lb(constraint_i_dim);
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
    result[2*constraint_dim] = X_iter.segment(3,3).squaredNorm()-1;
    result[2*constraint_dim+1] = -X_iter.segment(3,3).squaredNorm()+1;
    if (grad != NULL)
    {
        grad[(2*constraint_dim)*solver->num_variables_+0] = 0;
        grad[(2*constraint_dim)*solver->num_variables_+1] = 0;
        grad[(2*constraint_dim)*solver->num_variables_+2] = 0;
        grad[(2*constraint_dim)*solver->num_variables_+3] = 2*X_iter(3);
        grad[(2*constraint_dim)*solver->num_variables_+4] = 2*X_iter(4);
        grad[(2*constraint_dim)*solver->num_variables_+5] = 2*X_iter(5);
        grad[(2*constraint_dim)*solver->num_variables_+6] = 0;
        grad[(2*constraint_dim+1)*solver->num_variables_+0] = 0;
        grad[(2*constraint_dim+1)*solver->num_variables_+1] = 0;
        grad[(2*constraint_dim+1)*solver->num_variables_+2] = 0;
        grad[(2*constraint_dim+1)*solver->num_variables_+3] = -2*X_iter(3);
        grad[(2*constraint_dim+1)*solver->num_variables_+4] = -2*X_iter(4);
        grad[(2*constraint_dim+1)*solver->num_variables_+5] = -2*X_iter(5);
        grad[(2*constraint_dim+1)*solver->num_variables_+6] = 0;
    }
}

void KinematicSolverNLOpt::nonlinear_inequality_constraints_configuration(uint m, double* result, uint n, const double* x, double *grad, void* data)
{
    KinematicSolverNLOpt* solver = (KinematicSolverNLOpt*) data;
    ++solver->num_g_calls_;
    Eigen::VectorXd q_iter = Eigen::VectorXd::Map(x, n);
    Eigen::VectorXd X_iter = solver->solver_utils_->getTransformVectorFromQ(q_iter, solver->T_current_);
    std::size_t constraint_dim = 0;
    for(std::size_t constraint_id = 0;constraint_id < solver->constraints_.size();++constraint_id)
    {
        Eigen::VectorXd g_constraint;
        Eigen::MatrixXd g_constraint_derivative;
        Eigen::VectorXd lb;
        Eigen::VectorXd ub;
        lb = solver->constraints_[constraint_id]->getLowerBounds();
        ub = solver->constraints_[constraint_id]->getUpperBounds();
//        lb = solver->lb_[constraint_id];
//        ub = solver->ub_[constraint_id];
        if(solver->constraints_[constraint_id]->constraint_space == Constraint::Configuration)
        {
            g_constraint = solver->constraints_[constraint_id]->calculateConstraintValue(q_iter);
        }
        else if(solver->constraints_[constraint_id]->constraint_space == Constraint::Operational)
        {
            g_constraint = solver->constraints_[constraint_id]->calculateConstraintValue(X_iter);
        }
        if (grad != NULL)
        {
            if(solver->constraints_[constraint_id]->constraint_space == Constraint::Configuration)
            {
#ifdef NUMERICAL_DIFF
                  g_constraint_derivative = solver->solver_utils_->computeQConstraintDerivative(q_iter, solver->constraints_[constraint_id]);
#endif
            }
            else if(solver->constraints_[constraint_id]->constraint_space == Constraint::Operational)
            {
#ifdef NUMERICAL_DIFF
                g_constraint_derivative = solver->solver_utils_->computeXConstraintDerivative(q_iter, solver->constraints_[constraint_id], solver->T_current_);
#endif
            }
        }
        for(std::size_t constraint_i_dim = 0;constraint_i_dim < g_constraint.size();++constraint_i_dim, ++constraint_dim)
        {
            result[2*constraint_dim] = g_constraint[constraint_i_dim]-ub(constraint_i_dim);
            result[2*constraint_dim+1] = -1*g_constraint[constraint_i_dim]+lb(constraint_i_dim);
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

void PrioritizedKinematicSolverNLOpt::nonlinear_inequality_constraints_configuration_prioritized(uint m, double* result, uint n, const double* x, double *grad, void* data)
{
    PrioritizedKinematicSolverNLOpt* solver = (PrioritizedKinematicSolverNLOpt*) data;
    Eigen::VectorXd q_iter = Eigen::VectorXd::Map(x, n);
    Eigen::VectorXd X_iter = solver->solver_utils_->getTransformVectorFromQ(q_iter, solver->T_current_);
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
            lb(0) = solver->constraints_[constraint_id]->optimization_bound-1e-3;
            ub(0) = solver->constraints_[constraint_id]->optimization_bound+1e-3;
        }
        else
        {
            lb = solver->constraints_[constraint_id]->getLowerBounds();
            ub = solver->constraints_[constraint_id]->getUpperBounds();
        }

        if(solver->constraints_[constraint_id]->constraint_space == Constraint::Configuration)
        {
            g_constraint = solver->constraints_[constraint_id]->calculateConstraintValue(q_iter);
        }
        else if(solver->constraints_[constraint_id]->constraint_space == Constraint::Operational)
        {
            g_constraint = solver->constraints_[constraint_id]->calculateConstraintValue(X_iter);
        }
        if (grad != NULL)
        {
            if(solver->constraints_[constraint_id]->constraint_space == Constraint::Configuration)
            {
#ifdef NUMERICAL_DIFF
                  g_constraint_derivative = solver->solver_utils_->computeQConstraintDerivative(q_iter, solver->constraints_[constraint_id]);
#endif
            }
            else if(solver->constraints_[constraint_id]->constraint_space == Constraint::Operational)
            {
#ifdef NUMERICAL_DIFF
                g_constraint_derivative = solver->solver_utils_->computeXConstraintDerivative(q_iter, solver->constraints_[constraint_id], solver->T_current_);
#endif
            }
        }
        for(std::size_t constraint_i_dim = 0;constraint_i_dim < g_constraint.size();++constraint_i_dim, ++constraint_dim)
        {
            result[2*constraint_dim] = g_constraint[constraint_i_dim]-ub(constraint_i_dim);
            result[2*constraint_dim+1] = -1*g_constraint[constraint_i_dim]+lb(constraint_i_dim);
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

void DualArmKinematicSolverNLOpt::nonlinear_inequality_constraints_dual_configuration_prioritized(uint m, double* result, uint n, const double* x, double *grad, void* data)
{
    DualArmKinematicSolverNLOpt* solver = (DualArmKinematicSolverNLOpt*) data;
    ++solver->num_g_calls_;
    Eigen::VectorXd q_iter = Eigen::VectorXd::Map(x, n);
    Eigen::VectorXd q1_iter = q_iter.segment(0,solver->kinematics1_->getDof());
    Eigen::VectorXd q2_iter = q_iter.segment(solver->kinematics1_->getDof(),solver->kinematics2_->getDof());
    Eigen::VectorXd X_iter = solver->solver_utils_->getTransformVectorFromDualQ(q1_iter, q2_iter, solver->T1_current_, solver->T2_current_);
    Eigen::VectorXd X1_iter = X_iter.segment(0,7);
    Eigen::VectorXd X2_iter = X_iter.segment(7,7);
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
            lb(0) = solver->constraints_[constraint_id]->optimization_bound-1e-6;
            ub(0) = solver->constraints_[constraint_id]->optimization_bound+1e-6;
        }
        else
        {
            lb = solver->constraints_[constraint_id]->getLowerBounds();
            ub = solver->constraints_[constraint_id]->getUpperBounds();
        }

        if(solver->constraints_[constraint_id]->constraint_space == Constraint::Configuration)
        {
            g_constraint = solver->constraints_[constraint_id]->calculateConstraintValue(q_iter);
        }
        else if(solver->constraints_[constraint_id]->constraint_space == Constraint::Operational)
        {
            g_constraint = solver->constraints_[constraint_id]->calculateConstraintValue(X_iter);
        }
        if (grad != NULL)
        {
            if(solver->constraints_[constraint_id]->constraint_space == Constraint::Configuration)
            {
#ifdef NUMERICAL_DIFF
                  g_constraint_derivative = solver->solver_utils_->computeDualQConstraintDerivative(q_iter, solver->constraints_[constraint_id]);
#endif
            }
            else if(solver->constraints_[constraint_id]->constraint_space == Constraint::Operational)
            {
#ifdef NUMERICAL_DIFF
                g_constraint_derivative = solver->solver_utils_->computeDualXConstraintDerivative(q_iter, solver->constraints_[constraint_id], solver->T1_current_, solver->T2_current_);
#endif
            }
        }
        for(std::size_t constraint_i_dim = 0;constraint_i_dim < g_constraint.size();++constraint_i_dim, ++constraint_dim)
        {
            result[2*constraint_dim] = g_constraint[constraint_i_dim]-ub(constraint_i_dim);
            result[2*constraint_dim+1] = -1*g_constraint[constraint_i_dim]+lb(constraint_i_dim);
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
    for(std::size_t constraint_id = 0;constraint_id < solver->constraints_left_.size();++constraint_id)
    {
        if(solver->constraints_left_[constraint_id]->minimization_constraint && !solver->constraints_left_[constraint_id]->high_priority_constraint)
            continue;
        if(solver->constraints_left_[constraint_id]->low_priority_constraint)
            continue;
        Eigen::VectorXd g_constraint;
        Eigen::MatrixXd g_constraint_derivative;
        Eigen::VectorXd lb;
        Eigen::VectorXd ub;
        if(solver->constraints_left_[constraint_id]->high_priority_constraint)
        {
            lb(0) = solver->constraints_left_[constraint_id]->optimization_bound-1e-6;
            ub(0) = solver->constraints_left_[constraint_id]->optimization_bound+1e-6;
        }
        else
        {
            lb = solver->constraints_left_[constraint_id]->getLowerBounds();
            ub = solver->constraints_left_[constraint_id]->getUpperBounds();
        }

        if(solver->constraints_left_[constraint_id]->constraint_space == Constraint::Configuration)
        {
            g_constraint = solver->constraints_left_[constraint_id]->calculateConstraintValue(q1_iter);
        }
        else if(solver->constraints_left_[constraint_id]->constraint_space == Constraint::Operational)
        {
            g_constraint = solver->constraints_left_[constraint_id]->calculateConstraintValue(X1_iter);
        }
        if (grad != NULL)
        {
            if(solver->constraints_left_[constraint_id]->constraint_space == Constraint::Configuration)
            {
#ifdef NUMERICAL_DIFF
                g_constraint_derivative = solver->solver_utils_->computeQConstraintDerivative(q1_iter, solver->constraints_left_[constraint_id]);
#endif
            }
            else if(solver->constraints_left_[constraint_id]->constraint_space == Constraint::Operational)
            {
#ifdef NUMERICAL_DIFF
                g_constraint_derivative = solver->solver_utils_->computeXConstraintDerivative(q1_iter, solver->constraints_left_[constraint_id], solver->T1_current_, solver->kinematics1_);
#endif
            }
        }
        for(std::size_t constraint_i_dim = 0;constraint_i_dim < g_constraint.size();++constraint_i_dim, ++constraint_dim)
        {
            result[2*constraint_dim] = g_constraint[constraint_i_dim]-ub(constraint_i_dim);
            result[2*constraint_dim+1] = -1*g_constraint[constraint_i_dim]+lb(constraint_i_dim);
            if (grad != NULL)
            {
                for(std::size_t variable_id = 0;variable_id < solver->kinematics1_->getDof();++variable_id)
                {
                    grad[(2*constraint_dim)*solver->num_variables_+variable_id] = g_constraint_derivative(constraint_i_dim, variable_id);
                    grad[(2*constraint_dim+1)*solver->num_variables_+variable_id] = -1*g_constraint_derivative(constraint_i_dim, variable_id);
                }
            }
        }
    }
    for(std::size_t constraint_id = 0;constraint_id < solver->constraints_right_.size();++constraint_id)
    {
        if(solver->constraints_right_[constraint_id]->minimization_constraint && !solver->constraints_right_[constraint_id]->high_priority_constraint)
            continue;
        if(solver->constraints_right_[constraint_id]->low_priority_constraint)
            continue;
        Eigen::VectorXd g_constraint;
        Eigen::MatrixXd g_constraint_derivative;
        Eigen::VectorXd lb;
        Eigen::VectorXd ub;
        if(solver->constraints_right_[constraint_id]->high_priority_constraint)
        {
            lb(0) = solver->constraints_right_[constraint_id]->optimization_bound-1e-6;
            ub(0) = solver->constraints_right_[constraint_id]->optimization_bound+1e-6;
        }
        else
        {
            lb = solver->constraints_right_[constraint_id]->getLowerBounds();
            ub = solver->constraints_right_[constraint_id]->getUpperBounds();
        }

        if(solver->constraints_right_[constraint_id]->constraint_space == Constraint::Configuration)
        {
            g_constraint = solver->constraints_right_[constraint_id]->calculateConstraintValue(q2_iter);
        }
        else if(solver->constraints_right_[constraint_id]->constraint_space == Constraint::Operational)
        {
            g_constraint = solver->constraints_right_[constraint_id]->calculateConstraintValue(X2_iter);
        }
        if (grad != NULL)
        {
            if(solver->constraints_right_[constraint_id]->constraint_space == Constraint::Configuration)
            {
#ifdef NUMERICAL_DIFF
                g_constraint_derivative = solver->solver_utils_->computeQConstraintDerivative(q2_iter, solver->constraints_right_[constraint_id]);
#endif
            }
            else if(solver->constraints_right_[constraint_id]->constraint_space == Constraint::Operational)
            {
#ifdef NUMERICAL_DIFF
                g_constraint_derivative = solver->solver_utils_->computeXConstraintDerivative(q2_iter, solver->constraints_right_[constraint_id], solver->T2_current_, solver->kinematics2_);
#endif
            }
        }
        for(std::size_t constraint_i_dim = 0;constraint_i_dim < g_constraint.size();++constraint_i_dim, ++constraint_dim)
        {
            result[2*constraint_dim] = g_constraint[constraint_i_dim]-ub(constraint_i_dim);
            result[2*constraint_dim+1] = -1*g_constraint[constraint_i_dim]+lb(constraint_i_dim);
            if (grad != NULL)
            {
                for(std::size_t variable_id = 0;variable_id < solver->kinematics2_->getDof();++variable_id)
                {
                    grad[(2*constraint_dim)*solver->num_variables_+solver->kinematics1_->getDof()+variable_id] = g_constraint_derivative(constraint_i_dim, variable_id);
                    grad[(2*constraint_dim+1)*solver->num_variables_+solver->kinematics1_->getDof()+variable_id] = -1*g_constraint_derivative(constraint_i_dim, variable_id);
                }
            }
        }
    }
}

Eigen::VectorXd GeometricSolverNLOpt::solve()
{
    opt_->set_min_objective(GeometricSolverNLOpt::optimization_function_operational, this);
    std::vector<double> tolerances;
    for(std::size_t constraint_id = 0;constraint_id < constraints_.size();++constraint_id)
    {
        for(std::size_t constraint_i_dim = 0;constraint_i_dim < constraints_[constraint_id]->getNumConstraints();++constraint_i_dim)
        {
            tolerances.push_back(1e-6);
            tolerances.push_back(1e-6);
        }
    }
    tolerances.push_back(1e-6);
    tolerances.push_back(1e-6);
    opt_->set_stopval(1e-8);
    opt_->set_ftol_rel(1e-10);
//    opt_->set_maxtime(100e-3);
    opt_->add_inequality_mconstraint(GeometricSolverNLOpt::nonlinear_inequality_constraints_operational, this, tolerances);
    std::vector<double> x;
    x.resize(num_variables_);
    for(std::size_t variable_id = 0;variable_id < num_variables_;++variable_id)
        x[variable_id] = X_current_(variable_id);
    double f_val = 0;
    try
    {
        nlopt::result res = opt_->optimize(x, f_val);
//        std::cout << "optimization result:" << res << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }
    Eigen::VectorXd optimal_value = Eigen::VectorXd::Map(x.data(), x.size());
//    std::cout << "optimal value: " << optimal_value.transpose() << std::endl;
    return optimal_value;
}


KinematicSolverNLOpt::KinematicSolverNLOpt(std::vector<Constraint *> &constraints, Eigen::VectorXd &q_current, rl::mdl::Kinematic *kinematics):
    constraints_(constraints), q_current_(q_current), kinematics_(kinematics)
{
    kinematics_->setPosition(q_current_);
    kinematics_->forwardPosition();
    T_current_ = kinematics_->getOperationalPosition(0).matrix();
    X_current_ = vectorFromTransform(T_current_);
    num_variables_ = q_current.size();
    opt_ = new nlopt::opt(nlopt::LD_SLSQP, num_variables_);
    solver_utils_ = new SolverUtils(kinematics_);
}

Eigen::VectorXd KinematicSolverNLOpt::solve()
{
    std::vector<double> tolerances;
    for(std::size_t constraint_id = 0;constraint_id < constraints_.size();++constraint_id)
    {
        lb_.push_back(constraints_[constraint_id]->getLowerBounds());
        ub_.push_back(constraints_[constraint_id]->getUpperBounds());
        for(std::size_t constraint_i_dim = 0;constraint_i_dim < constraints_[constraint_id]->getNumConstraints();++constraint_i_dim)
        {
            tolerances.push_back(1e-6);
            tolerances.push_back(1e-6);
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

    opt_->set_min_objective(KinematicSolverNLOpt::optimization_function_configuration, this);
    opt_->set_stopval(1e-6);
    opt_->set_ftol_abs(1e-8);

    opt_->set_lower_bounds(lb_vec);
    opt_->set_upper_bounds(ub_vec);

//    opt_->set_maxtime(10e-3);
    opt_->add_inequality_mconstraint(KinematicSolverNLOpt::nonlinear_inequality_constraints_configuration, this, tolerances);
    std::vector<double> x;
    x.resize(num_variables_);
    for(std::size_t variable_id = 0;variable_id < num_variables_;++variable_id)
        x[variable_id] = q_current_(variable_id);
    double f_val = 0;
    try
    {
        opt_->optimize(x, f_val);
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }
    Eigen::VectorXd optimal_value = Eigen::VectorXd::Map(x.data(), x.size());
//    std::cout << "optimal value:" << optimal_value.transpose() << std::endl;
    std::cout << num_f_calls_ << ", " << num_g_calls_ << std::endl;
    Eigen::VectorXd q_iter = optimal_value;
    Eigen::VectorXd X_iter = solver_utils_->getTransformVectorFromQ(q_iter, T_current_);
    for(std::size_t constraint_id = 0;constraint_id < constraints_.size();++constraint_id)
    {
        Eigen::VectorXd g_constraint;
        Eigen::VectorXd lb = constraints_[constraint_id]->getLowerBounds();
        Eigen::VectorXd ub = constraints_[constraint_id]->getUpperBounds();
        if(constraints_[constraint_id]->constraint_space == Constraint::Configuration)
            g_constraint = constraints_[constraint_id]->calculateConstraintValue(q_iter);
        else
            g_constraint = constraints_[constraint_id]->calculateConstraintValue(X_iter);
        if(!constraints_[constraint_id]->minimization_constraint)
            std::cout << constraints_[constraint_id]->constraint_name << ":" << lb.transpose() << " < " << g_constraint.transpose() << " < " << ub.transpose() << std::endl;
        else
            std::cout << constraints_[constraint_id]->constraint_name << ":" << g_constraint.transpose() << std::endl;

    }
    return optimal_value;
}


PrioritizedKinematicSolverNLOpt::PrioritizedKinematicSolverNLOpt(std::vector<Constraint *> &constraints, Eigen::VectorXd &q_current, rl::mdl::Kinematic *kinematics):
    constraints_(constraints), q_current_(q_current), kinematics_(kinematics)
{
    kinematics_->setPosition(q_current_);
    kinematics_->forwardPosition();
    T_current_ = kinematics_->getOperationalPosition(0).matrix();
    X_current_ = vectorFromTransform(T_current_);
    num_variables_ = q_current.size();
    opt_ = new nlopt::opt(nlopt::LD_SLSQP, num_variables_);
    solver_utils_ = new SolverUtils(kinematics_);
}

Eigen::VectorXd PrioritizedKinematicSolverNLOpt::solve()
{
    std::vector<double> tolerances;
    for(std::size_t constraint_id = 0;constraint_id < constraints_.size();++constraint_id)
    {
        if(constraints_[constraint_id]->minimization_constraint && !constraints_[constraint_id]->high_priority_constraint)
            continue;
        if(constraints_[constraint_id]->low_priority_constraint)
            continue;
        for(std::size_t constraint_i_dim = 0;constraint_i_dim < constraints_[constraint_id]->getNumConstraints();++constraint_i_dim)
        {
            tolerances.push_back(1e-8);
            tolerances.push_back(1e-8);
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

    opt_->set_min_objective(PrioritizedKinematicSolverNLOpt::optimization_function_configuration_prioritized, this);
    opt_->set_stopval(1e-8);
    opt_->set_ftol_rel(1e-8);

    opt_->set_lower_bounds(lb_vec);
    opt_->set_upper_bounds(ub_vec);

    opt_->set_maxtime(10e-3);
    opt_->add_inequality_mconstraint(PrioritizedKinematicSolverNLOpt::nonlinear_inequality_constraints_configuration_prioritized, this, tolerances);
    std::vector<double> q;
    q.resize(num_variables_);
    for(std::size_t variable_id = 0;variable_id < num_variables_;++variable_id)
        q[variable_id] = q_current_(variable_id);
    double f_val = 0;
    try
    {
        opt_->optimize(q, f_val);
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }
    Eigen::VectorXd optimal_value = Eigen::VectorXd::Map(q.data(), q.size());
//    std::cout << "optimal value:" << optimal_value.transpose() << std::endl;
    return optimal_value;
}

DualArmKinematicSolverNLOpt::DualArmKinematicSolverNLOpt(std::vector<Constraint*> &constraints, std::vector<Constraint*> &constraints_left, std::vector<Constraint*> &constraints_right, Eigen::VectorXd &q1_current, Eigen::VectorXd &q2_current, rl::mdl::Kinematic *kinematics1, rl::mdl::Kinematic *kinematics2):
    constraints_(constraints), constraints_left_(constraints_left), constraints_right_(constraints_right), q1_current_(q1_current), q2_current_(q2_current), kinematics1_(kinematics1), kinematics2_(kinematics2)
{
    kinematics1_->setPosition(q1_current_);
    kinematics1_->forwardPosition();
    T1_current_ = kinematics1_->getOperationalPosition(0).matrix();
    X1_current_ = vectorFromTransform(T1_current_);
    kinematics2_->setPosition(q2_current_);
    kinematics2_->forwardPosition();
    T2_current_ = kinematics2_->getOperationalPosition(0).matrix();
    X2_current_ = vectorFromTransform(T2_current_);
    num_variables_ = q1_current.size()+q2_current.size();

    opt_ = new nlopt::opt(nlopt::LN_COBYLA, num_variables_);
    solver_utils_ = new SolverUtils(kinematics1_, kinematics2_);
}

Eigen::VectorXd DualArmKinematicSolverNLOpt::solve()
{
    std::vector<double> tolerances;
    for(std::size_t constraint_id = 0;constraint_id < constraints_.size();++constraint_id)
    {
        if(constraints_[constraint_id]->minimization_constraint && !constraints_[constraint_id]->high_priority_constraint)
            continue;
        if(constraints_[constraint_id]->low_priority_constraint)
            continue;
        for(std::size_t constraint_i_dim = 0;constraint_i_dim < constraints_[constraint_id]->getNumConstraints();++constraint_i_dim)
        {
            tolerances.push_back(1e-6);
            tolerances.push_back(1e-6);
        }
    }
    for(std::size_t constraint_id = 0;constraint_id < constraints_left_.size();++constraint_id)
    {
        if(constraints_left_[constraint_id]->minimization_constraint && !constraints_left_[constraint_id]->high_priority_constraint)
            continue;
        if(constraints_left_[constraint_id]->low_priority_constraint)
            continue;
        for(std::size_t constraint_i_dim = 0;constraint_i_dim < constraints_left_[constraint_id]->getNumConstraints();++constraint_i_dim)
        {
            tolerances.push_back(1e-6);
            tolerances.push_back(1e-6);
        }
    }
    for(std::size_t constraint_id = 0;constraint_id < constraints_right_.size();++constraint_id)
    {
        if(constraints_right_[constraint_id]->minimization_constraint && !constraints_right_[constraint_id]->high_priority_constraint)
            continue;
        if(constraints_right_[constraint_id]->low_priority_constraint)
            continue;
        for(std::size_t constraint_i_dim = 0;constraint_i_dim < constraints_right_[constraint_id]->getNumConstraints();++constraint_i_dim)
        {
            tolerances.push_back(1e-6);
            tolerances.push_back(1e-6);
        }
    }
    Eigen::VectorXd lb, ub;
    lb.resize(kinematics1_->getDof()+kinematics2_->getDof());
    ub.resize(kinematics1_->getDof()+kinematics2_->getDof());
    rl::math::Vector val;
    kinematics1_->getMinimum(val);
    lb.segment(0,kinematics1_->getDof()) = val;
    kinematics1_->getMaximum(val);
    ub.segment(0,kinematics1_->getDof()) = val;
    kinematics2_->getMinimum(val);
    lb.segment(kinematics1_->getDof(),kinematics2_->getDof()) = val;
    kinematics2_->getMaximum(val);
    ub.segment(kinematics1_->getDof(),kinematics2_->getDof()) = val;
    std::vector<double> ub_vec, lb_vec;
    for(std::size_t var_id = 0;var_id < ub.size();++var_id)
    {
        lb_vec.push_back(lb(var_id));
        ub_vec.push_back(ub(var_id));
    }

    opt_->set_min_objective(DualArmKinematicSolverNLOpt::optimization_function_dual_configuration_prioritized, this);
    opt_->set_stopval(1e-6);
    opt_->set_ftol_rel(1e-6);

    opt_->set_lower_bounds(lb_vec);
    opt_->set_upper_bounds(ub_vec);

//    opt_->set_maxtime(100e-3);
    opt_->add_inequality_mconstraint(DualArmKinematicSolverNLOpt::nonlinear_inequality_constraints_dual_configuration_prioritized, this, tolerances);
    std::vector<double> q;
    q.resize(num_variables_);
    for(std::size_t variable_id = 0;variable_id < q1_current_.size();++variable_id)
        q[variable_id] = q1_current_(variable_id);
    for(std::size_t variable_id = 0;variable_id < q2_current_.size();++variable_id)
        q[variable_id+q1_current_.size()] = q2_current_(variable_id);
    double f_val = 0;
    try
    {
        nlopt::result res = opt_->optimize(q, f_val);
        std::cout << "optimization result:" << res << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }
    Eigen::VectorXd optimal_value = Eigen::VectorXd::Map(q.data(), q.size());
    std::cout << "optimal function value:" << f_val << std::endl;
//    std::cout << "optimal value:" << rl::math::RAD2DEG*optimal_value.transpose() << std::endl;

    Eigen::VectorXd q_iter = optimal_value;
    Eigen::VectorXd q_iter_left = q_iter.segment(0,kinematics1_->getDof());
    Eigen::VectorXd q_iter_right = q_iter.segment(kinematics1_->getDof(),kinematics2_->getDof());
    Eigen::VectorXd X_iter = solver_utils_->getTransformVectorFromDualQ(q_iter.segment(0,kinematics1_->getDof()),q_iter.segment(kinematics1_->getDof(),kinematics2_->getDof()),T1_current_, T2_current_);
    Eigen::VectorXd X_iter_left = X_iter.segment(0,7);
    Eigen::VectorXd X_iter_right = X_iter.segment(7,7);
    for(std::size_t constraint_id = 0;constraint_id < constraints_.size();++constraint_id)
    {
        Eigen::VectorXd g_constraint;
        Eigen::VectorXd lb = constraints_[constraint_id]->getLowerBounds();
        Eigen::VectorXd ub = constraints_[constraint_id]->getUpperBounds();
        if(constraints_[constraint_id]->constraint_space == Constraint::Configuration)
            g_constraint = constraints_[constraint_id]->calculateConstraintValue(q_iter);
        else
            g_constraint = constraints_[constraint_id]->calculateConstraintValue(X_iter);
        if(!constraints_[constraint_id]->minimization_constraint)
            std::cout << constraints_[constraint_id]->constraint_name << ":" << lb.transpose() << " < " << g_constraint.transpose() << " < " << ub.transpose() << std::endl;
        else if (constraints_[constraint_id]->high_priority_constraint)
            std::cout << constraints_[constraint_id]->constraint_name << ":" << constraints_[constraint_id]->optimization_bound << " < " << g_constraint.transpose() << " < " << constraints_[constraint_id]->optimization_bound << std::endl;
        else
            std::cout << constraints_[constraint_id]->constraint_name << ":" << g_constraint.transpose() << std::endl;

    }
    for(std::size_t constraint_id = 0;constraint_id < constraints_left_.size();++constraint_id)
    {
        Eigen::VectorXd g_constraint;
        Eigen::VectorXd lb = constraints_left_[constraint_id]->getLowerBounds();
        Eigen::VectorXd ub = constraints_left_[constraint_id]->getUpperBounds();
        if(constraints_left_[constraint_id]->constraint_space == Constraint::Configuration)
            g_constraint = constraints_left_[constraint_id]->calculateConstraintValue(q_iter_left);
        else
            g_constraint = constraints_left_[constraint_id]->calculateConstraintValue(X_iter_left);
        if(!constraints_left_[constraint_id]->minimization_constraint)
            std::cout << constraints_left_[constraint_id]->constraint_name << ":" << lb.transpose() << " < " << g_constraint.transpose() << " < " << ub.transpose() << std::endl;
        else if (constraints_left_[constraint_id]->high_priority_constraint)
            std::cout << constraints_left_[constraint_id]->constraint_name << ":" << constraints_left_[constraint_id]->optimization_bound << " < " << g_constraint.transpose() << " < " << constraints_left_[constraint_id]->optimization_bound << std::endl;
        else
            std::cout << constraints_left_[constraint_id]->constraint_name << ":" << g_constraint.transpose() << std::endl;

    }
    for(std::size_t constraint_id = 0;constraint_id < constraints_right_.size();++constraint_id)
    {
        Eigen::VectorXd g_constraint;
        Eigen::VectorXd lb = constraints_right_[constraint_id]->getLowerBounds();
        Eigen::VectorXd ub = constraints_right_[constraint_id]->getUpperBounds();
        if(constraints_right_[constraint_id]->constraint_space == Constraint::Configuration)
            g_constraint = constraints_right_[constraint_id]->calculateConstraintValue(q_iter_right);
        else
            g_constraint = constraints_right_[constraint_id]->calculateConstraintValue(X_iter_right);
        if(!constraints_right_[constraint_id]->minimization_constraint)
            std::cout << constraints_right_[constraint_id]->constraint_name << ":" << lb.transpose() << " < " << g_constraint.transpose() << " < " << ub.transpose() << std::endl;
        else if (constraints_right_[constraint_id]->high_priority_constraint)
            std::cout << constraints_right_[constraint_id]->constraint_name << ":" << constraints_right_[constraint_id]->optimization_bound << " < " << g_constraint.transpose() << " < " << constraints_right_[constraint_id]->optimization_bound << std::endl;
        else
            std::cout << constraints_right_[constraint_id]->constraint_name << ":" << g_constraint.transpose() << std::endl;

    }
    std::cout << num_f_calls_ << ", " << num_g_calls_ << std::endl;

    return optimal_value;
}

