

#include "SolverUtils.h"

SolverUtils::SolverUtils(rl::mdl::Kinematic* &kin):
    kinematic(kin)
{
}

SolverUtils::SolverUtils(rl::mdl::Kinematic* &kin1, rl::mdl::Kinematic* &kin2):
    kin1(kin1), kin2(kin2)
{
}

Eigen::VectorXd SolverUtils::getTransformVectorFromQ(Eigen::VectorXd &q, Eigen::Matrix4d &T_current_, int ef_id)
{
    kinematic->setPosition(q);
    kinematic->forwardPosition();

    rl::math::Transform T = kinematic->getOperationalPosition(ef_id);
    T = (T.matrix())*(T_current_.inverse());
    Eigen::VectorXd X = quaternionFromTransform(T.matrix());
    return X;
}

Eigen::VectorXd SolverUtils::getTransformVectorFromQ(Eigen::VectorXd &q, std::vector<Eigen::Matrix4d> &T_current_)
{
    kinematic->setPosition(q);
    kinematic->forwardPosition();

    Eigen::VectorXd X;
    X.resize(7*kinematic->getOperationalDof());
    for(std::size_t ef_id = 0;ef_id < kinematic->getOperationalDof();++ef_id)
    {
        rl::math::Transform T = kinematic->getOperationalPosition(ef_id);
        T = (T.matrix())*(T_current_[ef_id].inverse());
        X.segment(7*ef_id,7) = quaternionFromTransform(T.matrix());
    }

    return X;
}

Eigen::VectorXd SolverUtils::getTransformVecFromQ(Eigen::VectorXd &q, Eigen::Matrix4d &T_current_, rl::mdl::Kinematic* &kin)
{
    kin->setPosition(q);
    kin->forwardPosition();

    rl::math::Transform T = kin->getOperationalPosition(0);
    T = (T.matrix())*(T_current_.inverse());
    Eigen::VectorXd X = quaternionFromTransform(T.matrix());
    return X;
}

Eigen::VectorXd SolverUtils::getTransformVecFromQAA(Eigen::VectorXd &q, Eigen::Matrix4d &T_current_, rl::mdl::Kinematic *&kin)
{
    kin->setPosition(q);
    kin->forwardPosition();

    rl::math::Transform T = kin->getOperationalPosition(0);
    T = (T.matrix())*(T_current_.inverse());
    Eigen::VectorXd X = vectorFromTransform(T.matrix());
    return X;
}

Eigen::VectorXd SolverUtils::getTransformVectorFromDualQ(Eigen::VectorXd q1, Eigen::VectorXd q2, Eigen::Matrix4d &T1_current_, Eigen::Matrix4d &T2_current_)
{
    kin1->setPosition(q1);
    kin1->forwardPosition();
    rl::math::Transform T1 = kin1->getOperationalPosition(0);
    T1 = (T1.matrix())*(T1_current_.inverse());
    Eigen::VectorXd X1 = vectorFromTransform(T1.matrix());

    kin2->setPosition(q2);
    kin2->forwardPosition();
    rl::math::Transform T2 = kin2->getOperationalPosition(0);
    T2 = (T2.matrix())*(T2_current_.inverse());
    Eigen::VectorXd X2 = vectorFromTransform(T2.matrix());

    Eigen::VectorXd X;
    X.resize(14);
    X.segment(0,7) = X1;
    X.segment(7,7) = X2;
    return X;
}

Eigen::MatrixXd SolverUtils::computeJacobianTransformVectorQ(Eigen::VectorXd &q, Eigen::Matrix4d &T_current_)
{
    Eigen::MatrixXd J;
    J.resize(7,q.size());
    Eigen::VectorXd X_minus, X_plus;
    X_minus.resize(7);
    X_plus.resize(7);
    Eigen::VectorXd q_minus, q_plus;
    q_minus.resize(q.size());
    q_plus.resize(q.size());
    double eps = 1e-8;
    for(std::size_t q_id = 0;q_id < q.size();++q_id)
    {
        q_minus = q;
        q_plus = q;
        q_minus(q_id) -= eps;
        q_plus(q_id) += eps;
        X_minus = getTransformVectorFromQ(q_minus, T_current_);
        X_plus = getTransformVectorFromQ(q_plus, T_current_);
        J.block(0,q_id,7,1) = (X_plus-X_minus)/(2.0*eps);
    }
    return J;
}

Eigen::MatrixXd SolverUtils::computeJacobianXQ(Eigen::VectorXd &q)
{
    Eigen::MatrixXd J;
    J.resize(7,q.size());
    Eigen::VectorXd X_minus, X_plus;
    X_minus.resize(7);
    X_plus.resize(7);
    Eigen::VectorXd q_minus, q_plus;
    q_minus.resize(q.size());
    q_plus.resize(q.size());
    double eps = 1e-8;
    Eigen::Matrix4d T_current_;
    T_current_.setIdentity();
    for(std::size_t q_id = 0;q_id < q.size();++q_id)
    {
        q_minus = q;
        q_plus = q;
        q_minus(q_id) -= eps;
        q_plus(q_id) += eps;
        X_minus = getTransformVectorFromQ(q_minus, T_current_);
        X_plus = getTransformVectorFromQ(q_plus, T_current_);
        J.block(0,q_id,7,1) = (X_plus-X_minus)/(2.0*eps);
    }
    return J;
}

Eigen::VectorXd SolverUtils::computeDerivativeCartDistanceQ(Eigen::VectorXd &q, rl::math::Transform &t_d)
{
    Eigen::VectorXd derivative_q;
    derivative_q.resize(q.size());
    Eigen::VectorXd q_minus, q_plus;
    q_minus.resize(q.size());
    q_plus.resize(q.size());
    double eps = 1e-7;
    for(std::size_t q_id = 0;q_id < q.size();++q_id)
    {
        q_minus = q;
        q_plus = q;
        q_minus(q_id) -= eps;
        q_plus(q_id) += eps;

        kinematic->setPosition(q_plus);
        kinematic->forwardPosition();
        rl::math::Transform t_plus = kinematic->getOperationalPosition(0);

        kinematic->setPosition(q_minus);
        kinematic->forwardPosition();
        rl::math::Transform t_minus = kinematic->getOperationalPosition(0);

        double dist1 = rl::math::transform::distance<rl::math::Transform, rl::math::Transform, double> (t_plus, t_d);
        double dist2 = rl::math::transform::distance<rl::math::Transform, rl::math::Transform, double> (t_minus, t_d);
//        double dist1 = t_plus.distance(t_d);
//        double dist2 = t_minus.distance(t_d);
        derivative_q(q_id) = (dist1-dist2)/(2.0*eps);
    }
    kinematic->setPosition(q);
    kinematic->forwardPosition();
    return derivative_q;
}

Eigen::VectorXd SolverUtils::computeDerivativeCartesianDistanceQ(Eigen::VectorXd &q, rl::math::Transform &t_d)
{
    Eigen::VectorXd derivative_q;
    derivative_q.resize(q.size());
    Eigen::VectorXd q_minus, q_plus;
    q_minus.resize(q.size());
    q_plus.resize(q.size());
    double eps = 1e-7;
    for(std::size_t q_id = 0;q_id < q.size();++q_id)
    {
        q_minus = q;
        q_plus = q;
        q_minus(q_id) -= eps;
        q_plus(q_id) += eps;

        kinematic->setPosition(q_plus);
        kinematic->forwardPosition();
        rl::math::Transform t_plus = kinematic->getOperationalPosition(0);

        kinematic->setPosition(q_minus);
        kinematic->forwardPosition();
        rl::math::Transform t_minus = kinematic->getOperationalPosition(0);

//        rl::math::Vector delta_minus = t_minus.toDelta(t_d);
        rl::math::Vector delta_minus(6);
        rl::math::transform::toDelta(t_minus, t_d, delta_minus);
//        if(std::fabs(delta_minus(0)) < 1e-6)
//            delta_minus(0) = 0;
//        if(std::fabs(delta_minus(1)) < 1e-6)
//            delta_minus(1) = 0;
//        if(std::fabs(delta_minus(2)) < 1e-6)
//            delta_minus(2) = 0;
//        if(std::fabs(delta_minus(3)) < 1e-6)
//            delta_minus(3) = 0;
//        if(std::fabs(delta_minus(4)) < 1e-6)
//            delta_minus(4) = 0;
//        if(std::fabs(delta_minus(5)) < 1e-6)
//            delta_minus(5) = 0;

//        rl::math::Vector delta_plus = t_plus.toDelta(t_d);
        rl::math::Vector delta_plus(6);
        rl::math::transform::toDelta(t_plus, t_d, delta_plus);
//        if(std::fabs(delta_plus(0)) < 1e-6)
//            delta_plus(0) = 0;
//        if(std::fabs(delta_plus(1)) < 1e-6)
//            delta_plus(1) = 0;
//        if(std::fabs(delta_plus(2)) < 1e-6)
//            delta_plus(2) = 0;
//        if(std::fabs(delta_plus(3)) < 1e-6)
//            delta_plus(3) = 0;
//        if(std::fabs(delta_plus(4)) < 1e-6)
//            delta_plus(4) = 0;
//        if(std::fabs(delta_plus(5)) < 1e-6)
//            delta_plus(5) = 0;

        derivative_q(q_id) = (delta_plus.squaredNorm()-delta_minus.squaredNorm())/(2.0*eps);
    }
    kinematic->setPosition(q);
    kinematic->forwardPosition();
    return derivative_q;
}

Eigen::MatrixXd SolverUtils::computeXConstraintDerivative(Eigen::VectorXd &q, Constraint* &constraint, Eigen::Matrix4d &T_current_)
{
    Eigen::MatrixXd J;
    J.resize(constraint->getNumConstraints(),q.size());
    Eigen::VectorXd val_minus, val_plus;
    Eigen::VectorXd q_minus, q_plus;
    q_minus.resize(q.size());
    q_plus.resize(q.size());
    Eigen::VectorXd X_minus;
    Eigen::VectorXd X_plus;
    double eps = 1e-7;
    for(std::size_t q_id = 0;q_id < q.size();++q_id)
    {
        q_minus = q;
        q_plus = q;
        q_minus(q_id) -= eps;
        q_plus(q_id) += eps;
        X_minus = getTransformVectorFromQ(q_minus, T_current_);
        X_plus = getTransformVectorFromQ(q_plus, T_current_);
        val_minus = constraint->calculateConstraintValue(X_minus);
        val_plus = constraint->calculateConstraintValue(X_plus);
        J.block(0,q_id,constraint->getNumConstraints(),1) = (val_plus-val_minus)/(2.0*eps);
    }
    return J;
}

Eigen::MatrixXd SolverUtils::computeXConstraintDerivative(Eigen::VectorXd &q, Constraint* &constraint, Eigen::Matrix4d &T_current_, rl::mdl::Kinematic* &kin)
{
    Eigen::MatrixXd J;
    J.resize(constraint->getNumConstraints(),q.size());
    Eigen::VectorXd val_minus, val_plus;
    Eigen::VectorXd q_minus, q_plus;
    q_minus.resize(q.size());
    q_plus.resize(q.size());
    Eigen::VectorXd X_minus;
    Eigen::VectorXd X_plus;
    double eps = 1e-7;
    for(std::size_t q_id = 0;q_id < q.size();++q_id)
    {
        q_minus = q;
        q_plus = q;
        q_minus(q_id) -= eps;
        q_plus(q_id) += eps;
        X_minus = getTransformVecFromQ(q_minus, T_current_, kin);
        X_plus = getTransformVecFromQ(q_plus, T_current_, kin);
        val_minus = constraint->calculateConstraintValue(X_minus);
        val_plus = constraint->calculateConstraintValue(X_plus);
        J.block(0,q_id,constraint->getNumConstraints(),1) = (val_plus-val_minus)/(2.0*eps);
    }
    return J;
}

Eigen::MatrixXd SolverUtils::computeXConstraintDerivative(Eigen::VectorXd &q, Constraint *&constraint, std::vector<Eigen::Matrix4d> &T_current_)
{
    Eigen::MatrixXd J;
    J.resize(constraint->getNumConstraints(),q.size());
    Eigen::VectorXd val_minus, val_plus;
    Eigen::VectorXd q_minus, q_plus;
    q_minus.resize(q.size());
    q_plus.resize(q.size());
    Eigen::VectorXd X_minus;
    Eigen::VectorXd X_plus;
    double eps = std::numeric_limits<float>::epsilon();
    for(std::size_t q_id = 0;q_id < q.size();++q_id)
    {
        q_minus = q;
        q_plus = q;
        q_minus(q_id) -= eps;
        q_plus(q_id) += eps;
        X_minus = getTransformVectorFromQ(q_minus, T_current_);
        X_plus = getTransformVectorFromQ(q_plus, T_current_);
        val_minus = constraint->calculateConstraintValue(X_minus);
        val_plus = constraint->calculateConstraintValue(X_plus);
        J.block(0,q_id,constraint->getNumConstraints(),1) = (val_plus-val_minus)/(2.0*eps);
    }
    return J;
}

Eigen::MatrixXd SolverUtils::computeXConstraintDerivativeAA(Eigen::VectorXd &q, OperationalPositionConstraint *&constraint, Eigen::Matrix4d &T_current_)
{
    Eigen::MatrixXd J;
    J.resize(constraint->getNumConstraints(),q.size());
    Eigen::VectorXd val_minus, val_plus;
    Eigen::VectorXd q_minus, q_plus;
    q_minus.resize(q.size());
    q_plus.resize(q.size());
    Eigen::VectorXd X_minus;
    Eigen::VectorXd X_plus;
    double eps = 1e-7;
    for(std::size_t q_id = 0;q_id < q.size();++q_id)
    {
        q_minus = q;
        q_plus = q;
        q_minus(q_id) -= eps;
        q_plus(q_id) += eps;
        X_minus = getTransformVectorFromQAA(q_minus, T_current_);
        X_plus = getTransformVectorFromQAA(q_plus, T_current_);
        val_minus = constraint->calculateConstraintValueAA(X_minus);
        val_plus = constraint->calculateConstraintValueAA(X_plus);
        J.block(0,q_id,constraint->getNumConstraints(),1) = (val_plus-val_minus)/(2.0*eps);
    }
    return J;
}

Eigen::MatrixXd SolverUtils::computeXConstraintDerivativeAA(Eigen::VectorXd &q, OperationalPositionConstraint *&constraint, Eigen::Matrix4d &T_current_, rl::mdl::Kinematic *&kin)
{
    Eigen::MatrixXd J;
    J.resize(constraint->getNumConstraints(),q.size());
    Eigen::VectorXd val_minus, val_plus;
    Eigen::VectorXd q_minus, q_plus;
    q_minus.resize(q.size());
    q_plus.resize(q.size());
    Eigen::VectorXd X_minus;
    Eigen::VectorXd X_plus;
    double eps = 1e-7;
    for(std::size_t q_id = 0;q_id < q.size();++q_id)
    {
        q_minus = q;
        q_plus = q;
        q_minus(q_id) -= eps;
        q_plus(q_id) += eps;
        X_minus = getTransformVecFromQAA(q_minus, T_current_, kin);
        X_plus = getTransformVecFromQAA(q_plus, T_current_, kin);
        val_minus = constraint->calculateConstraintValueAA(X_minus);
        val_plus = constraint->calculateConstraintValueAA(X_plus);
        J.block(0,q_id,constraint->getNumConstraints(),1) = (val_plus-val_minus)/(2.0*eps);
    }
    return J;
}

Eigen::MatrixXd SolverUtils::computeXConstraintDerivativeAA(Eigen::VectorXd &q, OperationalPositionConstraint *&constraint, std::vector<Eigen::Matrix4d> &T_current_)
{
    Eigen::MatrixXd J;
    J.resize(constraint->getNumConstraints(),q.size());
    Eigen::VectorXd val_minus, val_plus;
    Eigen::VectorXd q_minus, q_plus;
    q_minus.resize(q.size());
    q_plus.resize(q.size());
    Eigen::VectorXd X_minus;
    Eigen::VectorXd X_plus;
    double eps = 1e-7;
    for(std::size_t q_id = 0;q_id < q.size();++q_id)
    {
        q_minus = q;
        q_plus = q;
        q_minus(q_id) -= eps;
        q_plus(q_id) += eps;
        X_minus = getTransformVectorFromQAA(q_minus, T_current_);
        X_plus = getTransformVectorFromQAA(q_plus, T_current_);
        val_minus = constraint->calculateConstraintValueAA(X_minus);
        val_plus = constraint->calculateConstraintValueAA(X_plus);
        J.block(0,q_id,constraint->getNumConstraints(),1) = (val_plus-val_minus)/(2.0*eps);
    }
    return J;
}

Eigen::MatrixXd SolverUtils::computeQConstraintDerivative(Eigen::VectorXd &q, Constraint* &constraint)
{
    Eigen::MatrixXd J;
    J.resize(constraint->getNumConstraints(),q.size());
    Eigen::VectorXd val_minus, val_plus;
    Eigen::VectorXd q_minus, q_plus;
    q_minus.resize(q.size());
    q_plus.resize(q.size());
    double eps = std::numeric_limits<float>::epsilon();
    for(std::size_t q_id = 0;q_id < q.size();++q_id)
    {
        q_minus = q;
        q_plus = q;
        q_minus(q_id) -= eps;
        q_plus(q_id) += eps;
        val_minus = constraint->calculateConstraintValue(q_minus);
        val_plus = constraint->calculateConstraintValue(q_plus);
        J.block(0,q_id,constraint->getNumConstraints(),1) = (val_plus-val_minus)/(2.0*eps);
    }
    return J;
}

Eigen::MatrixXd SolverUtils::computeDualXConstraintDerivative(Eigen::VectorXd &q, Constraint* &constraint, Eigen::Matrix4d &T1_current_, Eigen::Matrix4d &T2_current_)
{
    Eigen::MatrixXd J;
    J.resize(constraint->getNumConstraints(),q.size());
    Eigen::VectorXd val_minus, val_plus;
    Eigen::VectorXd q_minus, q_plus;
    q_minus.resize(q.size());
    q_plus.resize(q.size());
    Eigen::VectorXd X_minus;
    Eigen::VectorXd X_plus;
    double eps = 1e-7;
    for(std::size_t q_id = 0;q_id < q.size();++q_id)
    {
        q_minus = q;
        q_plus = q;
        q_minus(q_id) -= eps;
        q_plus(q_id) += eps;
        X_minus = getTransformVectorFromDualQ(q_minus.segment(0, kin1->getDof()), q_minus.segment(kin1->getDof(),kin2->getDof()), T1_current_, T2_current_);
        X_plus = getTransformVectorFromDualQ(q_plus.segment(0, kin1->getDof()), q_plus.segment(kin1->getDof(),kin2->getDof()), T1_current_, T2_current_);
        val_minus = constraint->calculateConstraintValue(X_minus);
        val_plus = constraint->calculateConstraintValue(X_plus);
        J.block(0,q_id,constraint->getNumConstraints(),1) = (val_plus-val_minus)/(2.0*eps);
    }
    return J;
}

Eigen::MatrixXd SolverUtils::computeDualQConstraintDerivative(Eigen::VectorXd &q, Constraint* &constraint)
{
    Eigen::MatrixXd J;
    J.resize(constraint->getNumConstraints(),q.size());
    Eigen::VectorXd val_minus, val_plus;
    Eigen::VectorXd q_minus, q_plus;
    q_minus.resize(q.size());
    q_plus.resize(q.size());
    double eps = 1e-8;
    for(std::size_t q_id = 0;q_id < q.size();++q_id)
    {
        q_minus = q;
        q_plus = q;
        q_minus(q_id) -= eps;
        q_plus(q_id) += eps;
        val_minus = constraint->calculateConstraintValue(q_minus);
        val_plus = constraint->calculateConstraintValue(q_plus);
        J.block(0,q_id,constraint->getNumConstraints(),1) = (val_plus-val_minus)/(2.0*eps);
    }
    return J;
}


Eigen::VectorXd SolverUtils::getTransformVectorFromQAA(Eigen::VectorXd &q, Eigen::Matrix4d &T_current_, int ef_id)
{
    kinematic->setPosition(q);
    kinematic->forwardPosition();

    rl::math::Transform T = kinematic->getOperationalPosition(ef_id);
    T = (T.matrix())*(T_current_.inverse());
    Eigen::VectorXd X = vectorFromTransform(T.matrix());
    return X;
}

Eigen::VectorXd SolverUtils::getTransformVectorFromQAA(Eigen::VectorXd &q, std::vector<Eigen::Matrix4d> &T_current_)
{
    kinematic->setPosition(q);
    kinematic->forwardPosition();

    Eigen::VectorXd X;
    X.resize(7*kinematic->getOperationalDof());
    for(std::size_t ef_id = 0;ef_id < kinematic->getOperationalDof();++ef_id)
    {
        rl::math::Transform T = kinematic->getOperationalPosition(ef_id);
        T = (T.matrix())*(T_current_[ef_id].inverse());
        X.segment(7*ef_id,7) = vectorFromTransform(T.matrix());
    }

    return X;
}
