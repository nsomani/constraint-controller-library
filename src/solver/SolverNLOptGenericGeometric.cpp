

#include "SolverNLOptGenericGeometric.h"

Eigen::MatrixXd SolverNLOptGenericGeometricAA::calculateNumericalDerivativeGeometric(Constraint *&constraint, Eigen::VectorXd X_iter, Eigen::VectorXd X_current)
{
    Eigen::MatrixXd J;
    J.resize(constraint->getNumConstraints(),7);
    Eigen::VectorXd val_minus, val_plus;
    Eigen::VectorXd X_minus;
    Eigen::VectorXd X_plus;
    Eigen::VectorXd dX_minus;
    Eigen::VectorXd dX_plus;
    dX_minus.resize(X_iter.size());
    dX_plus.resize(X_iter.size());
    double eps = 1e-4;
    for(std::size_t dx_id = 0;dx_id < 7;++dx_id)
    {
        X_minus = X_iter;
        X_plus = X_iter;
        X_minus(dx_id) -= eps;
        X_plus(dx_id) += eps;
        X_minus.segment(3,3).normalize();
        X_plus.segment(3,3).normalize();
        for(std::size_t pose_id = 0;pose_id < X_iter.size()/7;++pose_id)
        {
            dX_minus.segment(pose_id*7,7) = getDelta(X_minus.segment(pose_id*7,7), X_current.segment(pose_id*7,7));
            dX_plus.segment(pose_id*7,7) = getDelta(X_plus.segment(pose_id*7,7), X_current.segment(pose_id*7,7));
        }
        val_minus = ((OperationalPositionConstraint*)constraint)->calculateConstraintValueAA(dX_minus);
        val_plus = ((OperationalPositionConstraint*)constraint)->calculateConstraintValueAA(dX_plus);
        J.block(0,dx_id,constraint->getNumConstraints(),1) = (val_plus-val_minus)/(2.0*eps);
    }
    return J;
}

double SolverNLOptGenericGeometricAA::optimization_function_operational(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    SolverNLOptGenericGeometricAA* solver = (SolverNLOptGenericGeometricAA*) data;
    Eigen::VectorXd X_iter = Eigen::VectorXd::Map(x.data(), x.size());
    Eigen::VectorXd val;
    val.resize(solver->num_variables_/7);
    for(std::size_t pose_id = 0;pose_id < solver->num_variables_/7;++pose_id)
    {
        val(pose_id) = distanceTTSS(transformFromVector(X_iter.segment(7*pose_id, 7)), transformFromVector(solver->X_current_.segment(7*pose_id, 7)));
    }

    if (!grad.empty())
    {
        Eigen::VectorXd dX_minus, dX_plus;
        dX_minus.resize(7);
        dX_plus.resize(7);
        for(std::size_t pose_id = 0;pose_id < solver->num_variables_/7;++pose_id)
        {
            double eps = 1e-4;
            for(std::size_t dx_id = 0;dx_id < 7;++dx_id)
            {
                dX_minus = X_iter.segment(7*pose_id, 7);
                dX_plus = X_iter.segment(7*pose_id, 7);
                dX_minus(dx_id) -= eps;
                dX_plus(dx_id) += eps;
                dX_minus.segment(3,3).normalize();
                dX_plus.segment(3,3).normalize();
                double val_minus = distanceTTSS(transformFromVector(dX_minus.segment(7*pose_id, 7)), transformFromVector(solver->X_current_.segment(7*pose_id, 7)));
                double val_plus = distanceTTSS(transformFromVector(dX_plus.segment(7*pose_id, 7)), transformFromVector(solver->X_current_.segment(7*pose_id, 7)));
                grad[pose_id*7+dx_id] = (val_plus*val_plus-val_minus*val_minus)/(2.0*eps);
            }
        }
    }
    return val.dot(val);
}

void SolverNLOptGenericGeometricAA::nonlinear_inequality_constraints_operational(uint m, double* result, uint n, const double* x, double *grad, void* data)
{
    SolverNLOptGenericGeometricAA* solver = (SolverNLOptGenericGeometricAA*) data;
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
            Eigen::VectorXd X_delta;
            if(solver->constraints_[constraint_id]->fixed_shape_id != solver->fixed_shape_id)
            {
                X_delta.resize(14);
                X_delta.segment(0,7) = getDelta(X_iter.segment(solver->constraints_[constraint_id]->constrained_shape_id*7,7), solver->X_current_.segment(solver->constraints_[constraint_id]->constrained_shape_id*7,7));
                X_delta.segment(7,7) = getDelta(X_iter.segment(solver->constraints_[constraint_id]->fixed_shape_id*7,7), solver->X_current_.segment(solver->constraints_[constraint_id]->fixed_shape_id*7,7));
            }
            else
            {
                X_delta.resize(7);
                X_delta = getDelta(X_iter.segment(solver->constraints_[constraint_id]->constrained_shape_id*7,7), solver->X_current_.segment(solver->constraints_[constraint_id]->constrained_shape_id*7,7));
//                std::cout << X_delta.transpose() << std::endl;
            }
            g_constraint = ((OperationalPositionConstraint*)solver->constraints_[constraint_id])->calculateConstraintValueAA(X_delta);
            lb = solver->constraints_[constraint_id]->getLowerBounds();
            ub = solver->constraints_[constraint_id]->getUpperBounds();
        }
        if (grad != NULL)
        {
            if(solver->constraints_[constraint_id]->constraint_space == Constraint::Operational)
            {
                Eigen::VectorXd X_delta;
                if(solver->constraints_[constraint_id]->fixed_shape_id != solver->fixed_shape_id)
                {
                    X_delta.resize(14);
                    X_delta.segment(0,7) = getDelta(X_iter.segment(solver->constraints_[constraint_id]->constrained_shape_id*7,7), solver->X_current_.segment(solver->constraints_[constraint_id]->constrained_shape_id*7,7));
                    X_delta.segment(7,7) = getDelta(X_iter.segment(solver->constraints_[constraint_id]->fixed_shape_id*7,7), solver->X_current_.segment(solver->constraints_[constraint_id]->fixed_shape_id*7,7));
                }
                else
                {
                    X_delta.resize(7);
                    X_delta = getDelta(X_iter.segment(solver->constraints_[constraint_id]->constrained_shape_id*7,7), solver->X_current_.segment(solver->constraints_[constraint_id]->constrained_shape_id*7,7));
                }
#ifdef NUMERICAL_DIFF
                g_constraint_derivative = solver->calculateNumericalDerivativeGeometric(solver->constraints_[constraint_id], X_iter, solver->X_current_);
#else
                g_constraint_derivative = solver->constraints_[constraint_id]->calculateConstraintDerivative(X_delta);
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
                    grad[(2*constraint_dim)*solver->num_variables_+variable_id] = 0;
                    grad[(2*constraint_dim+1)*solver->num_variables_+variable_id] = 0;
                }
                for(std::size_t variable_id = 0;variable_id < 7;++variable_id)
                {
                    grad[(2*constraint_dim)*solver->num_variables_+solver->constraints_[constraint_id]->constrained_shape_id*7+variable_id] = g_constraint_derivative(constraint_i_dim, variable_id);
                    grad[(2*constraint_dim+1)*solver->num_variables_+solver->constraints_[constraint_id]->constrained_shape_id*7+variable_id] = -1*g_constraint_derivative(constraint_i_dim, variable_id);
                }
            }
        }
    }
    for(std::size_t pose_id = 0;pose_id < solver->num_variables_/7;++pose_id)
    {
        result[2*constraint_dim+2*pose_id] = (X_iter.segment(7*pose_id+3,3)).squaredNorm()-1;
        result[2*constraint_dim+2*pose_id+1] = -1*(X_iter.segment(7*pose_id+3,3)).squaredNorm()+1;
        if (grad != NULL)
        {
            grad[(2*constraint_dim+2*pose_id)*solver->num_variables_+0] = 0;
            grad[(2*constraint_dim+2*pose_id)*solver->num_variables_+1] = 0;
            grad[(2*constraint_dim+2*pose_id)*solver->num_variables_+2] = 0;
            grad[(2*constraint_dim+2*pose_id)*solver->num_variables_+3] = 2*X_iter(7*pose_id+3);
            grad[(2*constraint_dim+2*pose_id)*solver->num_variables_+4] = 2*X_iter(7*pose_id+4);
            grad[(2*constraint_dim+2*pose_id)*solver->num_variables_+5] = 2*X_iter(7*pose_id+5);
            grad[(2*constraint_dim+2*pose_id)*solver->num_variables_+6] = 0;
            grad[(2*constraint_dim+2*pose_id+1)*solver->num_variables_+0] = 0;
            grad[(2*constraint_dim+2*pose_id+1)*solver->num_variables_+1] = 0;
            grad[(2*constraint_dim+2*pose_id+1)*solver->num_variables_+2] = 0;
            grad[(2*constraint_dim+2*pose_id+1)*solver->num_variables_+3] = -2*X_iter(7*pose_id+3);
            grad[(2*constraint_dim+2*pose_id+1)*solver->num_variables_+4] = -2*X_iter(7*pose_id+4);
            grad[(2*constraint_dim+2*pose_id+1)*solver->num_variables_+5] = -2*X_iter(7*pose_id+5);
            grad[(2*constraint_dim+2*pose_id+1)*solver->num_variables_+6] = 0;
        }
    }
}

SolverNLOptGenericGeometricAA::SolverNLOptGenericGeometricAA(std::vector<Constraint *> &constraints, Eigen::VectorXd &X_current):
    constraints_(constraints), X_current_(X_current), fixed_shape_id(-1)
{
    num_variables_ = X_current.size();
    opt_ = new nlopt::opt(nlopt::LN_COBYLA, num_variables_);
}

Eigen::VectorXd SolverNLOptGenericGeometricAA::solve()
{
    opt_->set_min_objective(SolverNLOptGenericGeometricAA::optimization_function_operational, this);
    std::vector<double> tolerances;
    for(std::size_t constraint_id = 0;constraint_id < constraints_.size();++constraint_id)
    {
        for(std::size_t constraint_i_dim = 0;constraint_i_dim < constraints_[constraint_id]->getNumConstraints();++constraint_i_dim)
        {
            tolerances.push_back(solver_params_.constraint_tol_);
            tolerances.push_back(solver_params_.constraint_tol_);
        }
    }
    for(std::size_t pose_id = 0;pose_id < num_variables_/7;++pose_id)
    {
        tolerances.push_back(solver_params_.constraint_tol_);
        tolerances.push_back(solver_params_.constraint_tol_);
    }

    std::vector<double> ub_vec, lb_vec;
    for(std::size_t pose_id = 0;pose_id < num_variables_/7;++pose_id)
    {
        for(std::size_t var_id = 0;var_id < 6;++var_id)
        {
            lb_vec.push_back(-1.0);
            ub_vec.push_back(1.0);
        }
        lb_vec.push_back(0);
        ub_vec.push_back(M_PI);
    }

    opt_->set_lower_bounds(lb_vec);
    opt_->set_upper_bounds(ub_vec);

//    opt_->set_stopval(solver_params_.stopval_);
    opt_->set_xtol_abs(solver_params_.x_tol_);
    opt_->set_maxtime(solver_params_.max_time_);

    opt_->add_inequality_mconstraint(SolverNLOptGenericGeometricAA::nonlinear_inequality_constraints_operational, this, tolerances);
    std::vector<double> x;
    x.resize(num_variables_);
    for(std::size_t variable_id = 0;variable_id < num_variables_;++variable_id)
    {

//        x[variable_id] = X_current_(variable_id);
        x[variable_id] += ((rand()%1000 - 500)/10000.0);
    }
    for(std::size_t pose_id = 0;pose_id < num_variables_/7;++pose_id)
    {
        double norm = std::sqrt(x[pose_id*7+3]*x[pose_id*7+3]+x[pose_id*7+4]*x[pose_id*7+4]+x[pose_id*7+5]*x[pose_id*7+5]);
        x[pose_id*7+3] /= norm;
        x[pose_id*7+4] /= norm;
        x[pose_id*7+5] /= norm;
    }
    double f_val = 0;
    try
    {
        nlopt::result res = opt_->optimize(x, f_val);
        std::cout << "optimization result:" << res << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }
    Eigen::VectorXd optimal_value = Eigen::VectorXd::Map(x.data(), x.size());
//    std::cout << "optimal value: " << optimal_value.transpose() << std::endl;
    return optimal_value;
}


double SolverNLOptGenericGeometric::optimization_function_operational(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    SolverNLOptGenericGeometric* solver = (SolverNLOptGenericGeometric*) data;
    Eigen::VectorXd X_iter = Eigen::VectorXd::Map(x.data(), x.size());
    Eigen::VectorXd val;
    val.resize(solver->num_variables_/7);
    for(std::size_t pose_id = 0;pose_id < solver->num_variables_/7;++pose_id)
    {
        val(pose_id) = distanceTTSS(transformFromQuaternion(X_iter.segment(7*pose_id, 7)), transformFromQuaternion(solver->X_current_.segment(7*pose_id, 7)));
    }

    if (!grad.empty())
    {
        Eigen::VectorXd dX_minus, dX_plus;
        dX_minus.resize(7);
        dX_plus.resize(7);
        for(std::size_t pose_id = 0;pose_id < solver->num_variables_/7;++pose_id)
        {
            double eps = 1e-6;
            for(std::size_t dx_id = 0;dx_id < 7;++dx_id)
            {
                dX_minus = X_iter.segment(7*pose_id, 7);
                dX_plus = X_iter.segment(7*pose_id, 7);
                dX_minus(dx_id) -= eps;
                dX_plus(dx_id) += eps;
                dX_minus.segment(3,4).normalize();
                dX_plus.segment(3,4).normalize();
                double val_minus = distanceTTSS(transformFromQuaternion(dX_minus.segment(7*pose_id, 7)), transformFromQuaternion(solver->X_current_.segment(7*pose_id, 7)));
                double val_plus = distanceTTSS(transformFromQuaternion(dX_plus.segment(7*pose_id, 7)), transformFromQuaternion(solver->X_current_.segment(7*pose_id, 7)));
                grad[pose_id*7+dx_id] = (val_plus*val_plus-val_minus*val_minus)/(2.0*eps);
            }
        }
    }
    return val.dot(val);

}

void SolverNLOptGenericGeometric::nonlinear_inequality_constraints_operational(uint m, double *result, uint n, const double *x, double *grad, void *data)
{
    SolverNLOptGenericGeometric* solver = (SolverNLOptGenericGeometric*) data;
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
            Eigen::VectorXd X_delta;
            if(solver->constraints_[constraint_id]->fixed_shape_id != solver->fixed_shape_id)
            {
                X_delta.resize(14);
                X_delta.segment(0,7) = getDeltaQuaternion(X_iter.segment(solver->pose_vec_map[solver->constraints_[constraint_id]->constrained_shape_id]*7,7), solver->X_current_.segment(solver->pose_vec_map[solver->constraints_[constraint_id]->constrained_shape_id]*7,7));
                X_delta.segment(7,7) = getDeltaQuaternion(X_iter.segment(solver->pose_vec_map[solver->constraints_[constraint_id]->fixed_shape_id]*7,7), solver->X_current_.segment(solver->pose_vec_map[solver->constraints_[constraint_id]->fixed_shape_id]*7,7));
            }
            else
            {
                X_delta.resize(7);
                X_delta = getDeltaQuaternion(X_iter.segment(solver->pose_vec_map[solver->constraints_[constraint_id]->constrained_shape_id]*7,7), solver->X_current_.segment(solver->pose_vec_map[solver->constraints_[constraint_id]->constrained_shape_id]*7,7));
            }
            g_constraint = solver->constraints_[constraint_id]->calculateConstraintValue(X_delta);
            lb = solver->constraints_[constraint_id]->getLowerBounds();
            ub = solver->constraints_[constraint_id]->getUpperBounds();
        }
        if (grad != NULL)
        {
            if(solver->constraints_[constraint_id]->constraint_space == Constraint::Operational)
            {
//                Eigen::VectorXd X_delta;
//                if(solver->constraints_[constraint_id]->fixed_shape_id != solver->fixed_shape_id)
//                {
//                    X_delta.resize(14);
//                    X_delta.segment(0,7) = getDeltaQuaternion(X_iter.segment(solver->pose_vec_map[solver->constraints_[constraint_id]->constrained_shape_id]*7,7), solver->X_current_.segment(solver->pose_vec_map[solver->constraints_[constraint_id]->constrained_shape_id]*7,7));
//                    X_delta.segment(7,7) = getDeltaQuaternion(X_iter.segment(solver->pose_vec_map[solver->constraints_[constraint_id]->fixed_shape_id]*7,7), solver->X_current_.segment(solver->pose_vec_map[solver->constraints_[constraint_id]->fixed_shape_id]*7,7));
//                }
//                else
//                {
//                    X_delta.resize(7);
//                    X_delta = getDeltaQuaternion(X_iter.segment(solver->pose_vec_map[solver->constraints_[constraint_id]->constrained_shape_id]*7,7), solver->X_current_.segment(solver->pose_vec_map[solver->constraints_[constraint_id]->constrained_shape_id]*7,7));
//                }
#ifdef NUMERICAL_DIFF
                g_constraint_derivative = solver->calculateNumericalDerivativeGeometric(solver->constraints_[constraint_id], X_iter, solver->X_current_);
#else
                g_constraint_derivative = solver->constraints_[constraint_id]->calculateConstraintDerivative(X_delta);
#endif
            }
        }
        for(std::size_t constraint_i_dim = 0;constraint_i_dim < g_constraint.size();++constraint_i_dim, ++constraint_dim)
        {
            result[2*constraint_dim] = g_constraint[constraint_i_dim]-ub(constraint_i_dim);
            result[2*constraint_dim+1] = -1*g_constraint[constraint_i_dim]+lb(constraint_i_dim);
            if (grad != NULL)
            {
//                for(std::size_t variable_id = 0;variable_id < solver->num_variables_;++variable_id)
//                {
//                    grad[(2*constraint_dim)*solver->num_variables_+variable_id] = 0;
//                    grad[(2*constraint_dim+1)*solver->num_variables_+variable_id] = 0;
//                }
                for(std::size_t variable_id = 0;variable_id < solver->num_variables_;++variable_id)
                {
                    grad[(2*constraint_dim)*solver->num_variables_+variable_id] = g_constraint_derivative(constraint_i_dim, variable_id);
                    grad[(2*constraint_dim+1)*solver->num_variables_+variable_id] = -1*g_constraint_derivative(constraint_i_dim, variable_id);
                }
            }
        }
    }
    for(std::size_t pose_id = 0;pose_id < solver->num_variables_/7;++pose_id)
    {
        result[2*constraint_dim+2*pose_id] = (X_iter.segment(7*pose_id+3,4)).squaredNorm()-1;
        result[2*constraint_dim+2*pose_id+1] = -1*(X_iter.segment(7*pose_id+3,4)).squaredNorm()+1;
        if (grad != NULL)
        {
            grad[(2*constraint_dim+2*pose_id)*solver->num_variables_+0] = 0;
            grad[(2*constraint_dim+2*pose_id)*solver->num_variables_+1] = 0;
            grad[(2*constraint_dim+2*pose_id)*solver->num_variables_+2] = 0;
            grad[(2*constraint_dim+2*pose_id)*solver->num_variables_+3] = 2*X_iter(7*pose_id+3);
            grad[(2*constraint_dim+2*pose_id)*solver->num_variables_+4] = 2*X_iter(7*pose_id+4);
            grad[(2*constraint_dim+2*pose_id)*solver->num_variables_+5] = 2*X_iter(7*pose_id+5);
            grad[(2*constraint_dim+2*pose_id)*solver->num_variables_+6] = 2*X_iter(7*pose_id+6);
            grad[(2*constraint_dim+2*pose_id+1)*solver->num_variables_+0] = 0;
            grad[(2*constraint_dim+2*pose_id+1)*solver->num_variables_+1] = 0;
            grad[(2*constraint_dim+2*pose_id+1)*solver->num_variables_+2] = 0;
            grad[(2*constraint_dim+2*pose_id+1)*solver->num_variables_+3] = -2*X_iter(7*pose_id+3);
            grad[(2*constraint_dim+2*pose_id+1)*solver->num_variables_+4] = -2*X_iter(7*pose_id+4);
            grad[(2*constraint_dim+2*pose_id+1)*solver->num_variables_+5] = -2*X_iter(7*pose_id+5);
            grad[(2*constraint_dim+2*pose_id+1)*solver->num_variables_+6] = -2*X_iter(7*pose_id+6);
        }
    }
}

SolverNLOptGenericGeometric::SolverNLOptGenericGeometric(std::vector<Constraint *> &constraints, Eigen::VectorXd &X_current):
    constraints_(constraints), X_current_(X_current), fixed_shape_id(0)
{
    num_variables_ = X_current.size();
    for(std::size_t pose_id = 0;pose_id < num_variables_/7;++pose_id)
    {
        pose_vec_map.insert(std::pair<int, int> (pose_id, pose_id));
    }
    opt_ = new nlopt::opt(nlopt::LD_SLSQP, num_variables_);
}

Eigen::VectorXd SolverNLOptGenericGeometric::solve()
{
    opt_->set_min_objective(SolverNLOptGenericGeometric::optimization_function_operational, this);
    std::vector<double> tolerances;
    for(std::size_t constraint_id = 0;constraint_id < constraints_.size();++constraint_id)
    {
        for(std::size_t constraint_i_dim = 0;constraint_i_dim < constraints_[constraint_id]->getNumConstraints();++constraint_i_dim)
        {
            tolerances.push_back(solver_params_.constraint_tol_);
            tolerances.push_back(solver_params_.constraint_tol_);
        }
    }
    for(std::size_t pose_id = 0;pose_id < num_variables_/7;++pose_id)
    {
        tolerances.push_back(solver_params_.constraint_tol_);
        tolerances.push_back(solver_params_.constraint_tol_);
    }

    std::vector<double> ub_vec, lb_vec;
    for(std::size_t pose_id = 0;pose_id < num_variables_/7;++pose_id)
    {
        for(std::size_t var_id = 0;var_id < 7;++var_id)
        {
            lb_vec.push_back(-1.0);
            ub_vec.push_back(1.0);
        }
    }

    opt_->set_lower_bounds(lb_vec);
    opt_->set_upper_bounds(ub_vec);

    opt_->set_stopval(solver_params_.stopval_);
    opt_->set_xtol_abs(solver_params_.x_tol_);
    opt_->set_maxtime(solver_params_.max_time_);

    opt_->add_inequality_mconstraint(SolverNLOptGenericGeometric::nonlinear_inequality_constraints_operational, this, tolerances);
    std::vector<double> x;
    x.resize(num_variables_);
    for(std::size_t variable_id = 0;variable_id < num_variables_;++variable_id)
    {
//        x[variable_id] = ((rand()%1000 - 500)/1000.0);
        x[variable_id] = X_current_(variable_id);
    }
    for(std::size_t pose_id = 0;pose_id < num_variables_/7;++pose_id)
    {
        double norm = std::sqrt(x[pose_id*7+3]*x[pose_id*7+3]+x[pose_id*7+4]*x[pose_id*7+4]+x[pose_id*7+5]*x[pose_id*7+5]+x[pose_id*7+6]*x[pose_id*7+6]);
        x[pose_id*7+3] /= norm;
        x[pose_id*7+4] /= norm;
        x[pose_id*7+5] /= norm;
        x[pose_id*7+6] /= norm;
    }
    double f_val = 0;
    try
    {
        nlopt::result res = opt_->optimize(x, f_val);
        std::cout << "optimization result:" << res << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }
    Eigen::VectorXd optimal_value = Eigen::VectorXd::Map(x.data(), x.size());
//    std::cout << "optimal value: " << optimal_value.transpose() << std::endl;

    Eigen::VectorXd X_iter = optimal_value;
    for(std::size_t constraint_id = 0;constraint_id < constraints_.size();++constraint_id)
    {
        Eigen::VectorXd g_constraint;
        Eigen::VectorXd lb = constraints_[constraint_id]->getLowerBounds();
        Eigen::VectorXd ub = constraints_[constraint_id]->getUpperBounds();
        Eigen::VectorXd X_delta;
        if(constraints_[constraint_id]->fixed_shape_id != fixed_shape_id)
        {
            X_delta.resize(14);
            X_delta.segment(0,7) = getDeltaQuaternion(X_iter.segment(pose_vec_map[constraints_[constraint_id]->constrained_shape_id]*7,7), X_current_.segment(pose_vec_map[constraints_[constraint_id]->constrained_shape_id]*7,7));
            X_delta.segment(7,7) = getDeltaQuaternion(X_iter.segment(pose_vec_map[constraints_[constraint_id]->fixed_shape_id]*7,7), X_current_.segment(pose_vec_map[constraints_[constraint_id]->fixed_shape_id]*7,7));
        }
        else
        {
            X_delta.resize(7);
            X_delta = getDeltaQuaternion(X_iter.segment(pose_vec_map[constraints_[constraint_id]->constrained_shape_id]*7,7), X_current_.segment(pose_vec_map[constraints_[constraint_id]->constrained_shape_id]*7,7));
        }
        g_constraint = constraints_[constraint_id]->calculateConstraintValue(X_delta);
        std::cout << constraints_[constraint_id]->constraint_name << ":" << lb.transpose() << " < " << g_constraint.transpose() << " < " << ub.transpose() << std::endl;
    }
    return optimal_value;
}

Eigen::MatrixXd SolverNLOptGenericGeometric::calculateNumericalDerivativeGeometric(Constraint *&constraint, Eigen::VectorXd X_iter, Eigen::VectorXd X_current)
{
    Eigen::MatrixXd J;
    J.resize(constraint->getNumConstraints(), X_iter.size());
    J.setZero();
    Eigen::VectorXd val_minus, val_plus;
    Eigen::VectorXd X_minus;
    Eigen::VectorXd X_plus;
    Eigen::VectorXd dX_minus;
    Eigen::VectorXd dX_plus;
    double eps = 1e-6;
    for(std::size_t pose_id = 0;pose_id < X_iter.size()/7;++pose_id)
    {
        if(pose_id != pose_vec_map[constraint->constrained_shape_id] && pose_id != pose_vec_map[constraint->fixed_shape_id])
            continue;
        for(std::size_t dx_id = 0;dx_id < 7;++dx_id)
        {
            X_minus = X_iter;
            X_plus = X_iter;
            X_minus(pose_id*7+dx_id) -= eps;
            X_plus(pose_id*7+dx_id) += eps;
            X_minus.segment(pose_id*7+3,4).normalize();
            X_plus.segment(pose_id*7+3,4).normalize();
            if(constraint->fixed_shape_id != fixed_shape_id)
            {
                dX_plus.resize(14);
                dX_plus.segment(0,7) = getDeltaQuaternion(X_plus.segment(pose_vec_map[constraint->constrained_shape_id]*7,7), X_current.segment(pose_vec_map[constraint->constrained_shape_id]*7,7));
                dX_plus.segment(7,7) = getDeltaQuaternion(X_plus.segment(pose_vec_map[constraint->fixed_shape_id]*7,7), X_current.segment(pose_vec_map[constraint->fixed_shape_id]*7,7));
                dX_minus.resize(14);
                dX_minus.segment(0,7) = getDeltaQuaternion(X_minus.segment(pose_vec_map[constraint->constrained_shape_id]*7,7), X_current.segment(pose_vec_map[constraint->constrained_shape_id]*7,7));
                dX_minus.segment(7,7) = getDeltaQuaternion(X_minus.segment(pose_vec_map[constraint->fixed_shape_id]*7,7), X_current.segment(pose_vec_map[constraint->fixed_shape_id]*7,7));
            }
            else
            {
                dX_plus.resize(7);
                dX_plus = getDeltaQuaternion(X_plus.segment(pose_vec_map[constraint->constrained_shape_id]*7,7), X_current.segment(pose_vec_map[constraint->constrained_shape_id]*7,7));
                dX_minus.resize(7);
                dX_minus = getDeltaQuaternion(X_minus.segment(pose_vec_map[constraint->constrained_shape_id]*7,7), X_current.segment(pose_vec_map[constraint->constrained_shape_id]*7,7));
            }
            val_minus = constraint->calculateConstraintValue(dX_minus);
            val_plus = constraint->calculateConstraintValue(dX_plus);
            J.block(0,pose_id*7+dx_id,constraint->getNumConstraints(),1) = (val_plus-val_minus)/(2.0*eps);
        }
    }
    return J;
}
