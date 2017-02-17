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


#ifndef SOLVERNLOPTGENERIC_H
#define SOLVERNLOPTGENERIC_H

#include "SolverNLOpt.h"
#include "../constraint/ConstraintUtils.h"

class SolverNLOptGenericKinematic: public SolverNLOpt
{
public:
    static double optimization_function_configuration_prioritized(const std::vector<double> &q, std::vector<double> &grad, void *data);
    static void nonlinear_inequality_constraints_configuration_prioritized(uint m, double* result, uint n, const double* x, double *grad, void* data);
    SolverNLOptGenericKinematic(std::vector<Constraint *> &constraints, Eigen::VectorXd q_current, rl::mdl::Kinematic *kinematics);
    SolverNLOptGenericKinematic(std::vector<Constraint *> &constraints, Eigen::VectorXd q_current, rl::mdl::Kinematic *kinematics, std::string solver_name);
    Eigen::VectorXd solve();
    Eigen::VectorXd solveResult(bool &result);
    Eigen::VectorXd getConstraintErrors(Eigen::VectorXd optimal_value);
    std::vector<Constraint*> constraints_;
    int num_variables_;
    std::vector<Eigen::Matrix4d> T_current_;
    Eigen::VectorXd X_sol_;
    Eigen::VectorXd X_current_;
    Eigen::VectorXd q_sol_;
    Eigen::VectorXd q_current_;
    rl::mdl::Kinematic *kinematics_;
    SolverUtils *solver_utils_;
    bool use_quaternions;
};

#endif // SOLVERNLOPTGENERIC_H
