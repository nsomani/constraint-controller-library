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


#ifndef SOLVERUTILS_H
#define SOLVERUTILS_H

#include <rl/hal/Coach.h>
#include <rl/math/Rotation.h>
#include <rl/mdl/Dynamic.h>
#include <rl/mdl/Model.h>
#include <rl/mdl/XmlFactory.h>
#include <rl/mdl/Body.h>

#include "../constraint/ConstraintUtils.h"
#include "../constraint/ConstraintUtils.h"
#include "../constraint/GeometricConstraints.h"
#include "../constraint/RobotConstraints.h"

class SolverUtils
{
public:
    SolverUtils(rl::mdl::Kinematic* &kin);
    SolverUtils(rl::mdl::Kinematic* &kin1, rl::mdl::Kinematic* &kin2);
    Eigen::VectorXd getEFPoseFromQ(Eigen::VectorXd &q);
    Eigen::VectorXd getTransformVectorFromQ(Eigen::VectorXd &q, Eigen::Matrix4d &T_current_, int ef_id = 0);
    Eigen::VectorXd getTransformVectorFromQ(Eigen::VectorXd &q, std::vector<Eigen::Matrix4d> &T_current_);
    Eigen::VectorXd getTransformVectorFromQAA(Eigen::VectorXd &q, Eigen::Matrix4d &T_current_, int ef_id = 0);
    Eigen::VectorXd getTransformVectorFromQAA(Eigen::VectorXd &q, std::vector<Eigen::Matrix4d> &T_current_);
    Eigen::VectorXd getTransformVecFromQ(Eigen::VectorXd &q, Eigen::Matrix4d &T_current_, rl::mdl::Kinematic* &kin);
    Eigen::VectorXd getTransformVecFromQAA(Eigen::VectorXd &q, Eigen::Matrix4d &T_current_, rl::mdl::Kinematic* &kin);
    Eigen::MatrixXd computeJacobianTransformVectorQ(Eigen::VectorXd &q, Eigen::Matrix4d &T_current_);
    Eigen::MatrixXd computeJacobianXQ(Eigen::VectorXd &q);
    Eigen::MatrixXd computeXConstraintDerivative(Eigen::VectorXd &q, Constraint *&constraint, Eigen::Matrix4d &T_current_);
    Eigen::MatrixXd computeXConstraintDerivative(Eigen::VectorXd &q, Constraint* &constraint, Eigen::Matrix4d &T_current_, rl::mdl::Kinematic* &kin);
    Eigen::MatrixXd computeXConstraintDerivative(Eigen::VectorXd &q, Constraint* &constraint, std::vector<Eigen::Matrix4d> &T_current_);
    Eigen::MatrixXd computeXConstraintDerivativeAA(Eigen::VectorXd &q, OperationalPositionConstraint *&constraint, Eigen::Matrix4d &T_current_);
    Eigen::MatrixXd computeXConstraintDerivativeAA(Eigen::VectorXd &q, OperationalPositionConstraint* &constraint, Eigen::Matrix4d &T_current_, rl::mdl::Kinematic* &kin);
    Eigen::MatrixXd computeXConstraintDerivativeAA(Eigen::VectorXd &q, OperationalPositionConstraint* &constraint, std::vector<Eigen::Matrix4d> &T_current_);
    Eigen::MatrixXd computeQConstraintDerivative(Eigen::VectorXd &q, Constraint *&constraint);
    Eigen::VectorXd computeDerivativeCartDistanceQ(Eigen::VectorXd &q, rl::math::Transform &t_d);
    Eigen::VectorXd computeDerivativeCartesianDistanceQ(Eigen::VectorXd &q, rl::math::Transform &t_d);
    Eigen::VectorXd getTransformVectorFromDualQ(Eigen::VectorXd q1, Eigen::VectorXd q2, Eigen::Matrix4d &T1_current_, Eigen::Matrix4d &T2_current_);

    Eigen::MatrixXd computeDualQConstraintDerivative(Eigen::VectorXd &q, Constraint* &constraint);
    Eigen::MatrixXd computeDualXConstraintDerivative(Eigen::VectorXd &q, Constraint* &constraint, Eigen::Matrix4d &T1_current_, Eigen::Matrix4d &T2_current_);

private:
    rl::mdl::Kinematic* kinematic;
    rl::mdl::Kinematic* kin1;
    rl::mdl::Kinematic* kin2;

};

#endif // SOLVERUTILS_H
