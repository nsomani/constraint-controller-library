

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
