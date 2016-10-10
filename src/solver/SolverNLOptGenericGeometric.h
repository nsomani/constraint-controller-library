

#ifndef SOLVERNLOPTGENERICGEOMETRIC_H
#define SOLVERNLOPTGENERICGEOMETRIC_H

#include "SolverNLOpt.h"
#include "../constraint/ConstraintUtils.h"

class SolverNLOptGenericGeometricAA : public SolverNLOpt
{
public:
    static double optimization_function_operational(const std::vector<double> &x, std::vector<double> &grad, void *data);
    static void nonlinear_inequality_constraints_operational(uint m, double* result, uint n, const double* x, double *grad, void* data);
    SolverNLOptGenericGeometricAA(std::vector<Constraint*> &constraints, Eigen::VectorXd &X_current);
    Eigen::VectorXd solve();
    Eigen::MatrixXd calculateNumericalDerivativeGeometric(Constraint* &constraint, Eigen::VectorXd X_iter, Eigen::VectorXd X_current);
    std::vector<Constraint*> constraints_;
    int num_variables_;
    Eigen::VectorXd X_;
    Eigen::VectorXd X_current_;
    int fixed_shape_id;
};

class SolverNLOptGenericGeometric : public SolverNLOpt
{
public:
    static double optimization_function_operational(const std::vector<double> &x, std::vector<double> &grad, void *data);
    static void nonlinear_inequality_constraints_operational(uint m, double* result, uint n, const double* x, double *grad, void* data);
    SolverNLOptGenericGeometric(std::vector<Constraint*> &constraints, Eigen::VectorXd &X_current);
    Eigen::VectorXd solve();
    Eigen::MatrixXd calculateNumericalDerivativeGeometric(Constraint* &constraint, Eigen::VectorXd X_iter, Eigen::VectorXd X_current);
    std::vector<Constraint*> constraints_;
    std::map<int, int> pose_vec_map;
    int num_variables_;
    Eigen::VectorXd X_;
    Eigen::VectorXd X_current_;
    int fixed_shape_id;
};


#endif // SOLVERNLOPTGENERICGEOMETRIC_H
