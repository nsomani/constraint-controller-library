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


#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include <stdio.h>
#include <iostream>
#include "ConstraintUtils.h"
#include "ObjectModel.h"

#include <limits>
#include <cmath>
#include <limits>
//#define FLOAT_EPS std::numeric_limits<double>::epsilon()

//#define USE_QUATERNIONS
//#define USE_AXISANGLE

class Constraint
{
public:
    enum Space {Configuration, Operational};
    enum Level {Position, Velocity, Acceleration};

    bool minimization_constraint;
    bool low_priority_constraint;
    bool high_priority_constraint;
    int priority_level;
    double optimization_bound;
    std::string constraint_name;
    Space constraint_space;
    Level constraint_level;
    int constrained_shape_id;
    int fixed_shape_id;
    Constraint(Space space, Level level, std::string name, bool minimization=false, int priority=1);
    virtual int getNumConstraints() = 0;
    virtual Eigen::VectorXd calculateConstraintValue(Eigen::VectorXd &vec) = 0;
    virtual Eigen::MatrixXd calculateConstraintDerivative(Eigen::VectorXd &vec) = 0;
    virtual Eigen::VectorXd getLowerBounds() = 0;
    virtual Eigen::VectorXd getUpperBounds() = 0;
    bool isConstraintSatisfied(Eigen::VectorXd &vec);
};

class JointPositionConstraint: public Constraint
{
protected:
    JointPositionConstraint(std::string name);
};

class JointVelocityConstraint: public Constraint
{
protected:
    JointVelocityConstraint(std::string name);
};

class JointAccelerationConstraint: public Constraint
{
protected:
    JointAccelerationConstraint(std::string name);
};

class OperationalPositionConstraint: public Constraint
{
public:
    virtual Eigen::VectorXd calculateConstraintValueAA(Eigen::VectorXd &vec) = 0;
    virtual Eigen::MatrixXd calculateConstraintDerivativeAA(Eigen::VectorXd &vec) = 0;
    virtual bool isConstraintSatisfied(Eigen::VectorXd &vec, bool use_quaternions=true);
protected:
    OperationalPositionConstraint(std::string name);

};

class OperationalVelocityConstraint: public Constraint
{
protected:
    OperationalVelocityConstraint(std::string name);
};

class OperationalAccelerationConstraint: public Constraint
{
protected:
    OperationalAccelerationConstraint(std::string name);
};

#endif // CONSTRAINT_H
