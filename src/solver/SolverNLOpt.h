

#ifndef SOLVERNLOPT_H
#define SOLVERNLOPT_H
#include <nlopt.hpp>
#include <stdio.h>
#include <iostream>
#include "../constraint/ConstraintUtils.h"
#include "../constraint/GeometricConstraints.h"
#include "SolverUtils.h"
//#include <Eigen/Dense>

class SolverParams
{
public:
    SolverParams();
    double f_tol_;
    double x_tol_;
    double constraint_tol_;
    double max_time_;
    double stopval_;
};

class SolverNLOpt
{
public:
    SolverNLOpt();
    virtual Eigen::VectorXd solve() = 0;
    void setSolverParams(SolverParams &params);
    SolverParams solver_params_;
protected:
    nlopt::opt *opt_;
    int num_f_calls_;
    int num_g_calls_;
    std::vector<Eigen::VectorXd> lb_;
    std::vector<Eigen::VectorXd> ub_;    
};


#define ANALYTICAL_DIFF
#define NUMERICAL_DIFF
class GeometricSolverNLOpt : public SolverNLOpt
{
public:
    GeometricSolverNLOpt(std::vector<Constraint*> &constraints, Eigen::VectorXd &X_current);
    static double optimization_function_operational(const std::vector<double> &x, std::vector<double> &grad, void *data);
    static void nonlinear_inequality_constraints_operational(uint m, double* result, uint n, const double* x, double *grad, void* data);
    Eigen::VectorXd solve();
    std::vector<Constraint*> constraints_;
    int num_variables_;
    Eigen::VectorXd X_;
    Eigen::VectorXd X_current_;

};

#include "../constraint/RobotConstraints.h"
class KinematicSolverNLOpt : public SolverNLOpt
{
public:
    KinematicSolverNLOpt(std::vector<Constraint*> &constraints, Eigen::VectorXd &q_current, rl::mdl::Kinematic *kinematics);
    static double optimization_function_configuration(const std::vector<double> &q, std::vector<double> &grad, void *data);
    static void nonlinear_inequality_constraints_configuration(uint m, double* result, uint n, const double* x, double *grad, void* data);
    Eigen::VectorXd solve();
    std::vector<Constraint*> constraints_;
    int num_variables_;
    Eigen::Matrix4d T_current_;
    Eigen::VectorXd X_;
    Eigen::VectorXd X_current_;
    Eigen::VectorXd q_;
    Eigen::VectorXd q_current_;
    rl::mdl::Kinematic *kinematics_;
    SolverUtils *solver_utils_;
};

#include "../constraint/RobotConstraints.h"
class PrioritizedKinematicSolverNLOpt : public SolverNLOpt
{
public:
    PrioritizedKinematicSolverNLOpt(std::vector<Constraint*> &constraints, Eigen::VectorXd &q_current, rl::mdl::Kinematic *kinematics);
    static double optimization_function_configuration_prioritized(const std::vector<double> &q, std::vector<double> &grad, void *data);
    static void nonlinear_inequality_constraints_configuration_prioritized(uint m, double* result, uint n, const double* x, double *grad, void* data);
    Eigen::VectorXd solve();
    std::vector<Constraint*> constraints_;
    int num_variables_;
    Eigen::Matrix4d T_current_;
    Eigen::VectorXd X_;
    Eigen::VectorXd X_current_;
    Eigen::VectorXd q_;
    Eigen::VectorXd q_current_;
    rl::mdl::Kinematic *kinematics_;
    SolverUtils *solver_utils_;
};

#include "../constraint/RobotConstraints.h"
class DualArmKinematicSolverNLOpt : public SolverNLOpt
{
public:
    DualArmKinematicSolverNLOpt(std::vector<Constraint*> &constraints, std::vector<Constraint*> &constraints_left, std::vector<Constraint*> &constraints_right, Eigen::VectorXd &q1_current, Eigen::VectorXd &q2_current, rl::mdl::Kinematic *kinematics1, rl::mdl::Kinematic *kinematics2);
    static double optimization_function_dual_configuration_prioritized(const std::vector<double> &q, std::vector<double> &grad, void *data);
    static void nonlinear_inequality_constraints_dual_configuration_prioritized(uint m, double* result, uint n, const double* x, double *grad, void* data);
    Eigen::VectorXd solve();
    std::vector<Constraint*> constraints_;
    std::vector<Constraint*> constraints_left_;
    std::vector<Constraint*> constraints_right_;
    int num_variables_;
    Eigen::Matrix4d T1_current_;
    Eigen::VectorXd X1_;
    Eigen::VectorXd X1_current_;
    Eigen::Matrix4d T2_current_;
    Eigen::VectorXd X2_;
    Eigen::VectorXd X2_current_;
    Eigen::VectorXd q1_;
    Eigen::VectorXd q1_current_;
    Eigen::VectorXd q2_;
    Eigen::VectorXd q2_current_;
    rl::mdl::Kinematic *kinematics1_, *kinematics2_;
    SolverUtils *solver_utils_;    
};

#endif // SOLVERNLOPT_H
