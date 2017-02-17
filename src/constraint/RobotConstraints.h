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


#ifndef ROBOTCONSTRAINTS_H
#define ROBOTCONSTRAINTS_H

#include "Constraint.h"
#include <rl/math/Rotation.h>
#include <rl/mdl/Dynamic.h>
#include <rl/mdl/Model.h>
#include <rl/mdl/Body.h>

class RobotConstraints: public Constraint
{
public:
    RobotConstraints(rl::mdl::Kinematic *kin);
    virtual int getNumConstraints();
    virtual Eigen::VectorXd calculateConstraintValue(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivative(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd getLowerBounds();
    virtual Eigen::VectorXd getUpperBounds();
    virtual void updateConstraintParams();

private:
    rl::mdl::Kinematic* kinematic;
};

class RobotVelocityLimitconstraint: public OperationalVelocityConstraint
{
public:
    RobotVelocityLimitconstraint(rl::mdl::Kinematic *kin);
    virtual int getNumConstraints();
    virtual Eigen::VectorXd calculateConstraintValue(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivative(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd getLowerBounds();
    virtual Eigen::VectorXd getUpperBounds();
    virtual void updateConstraintParams();

    rl::math::Transform EFpos_curr;
    rl::math::MotionVector EFvel_curr;
    rl::math::Transform EFpos_d;
    rl::math::MotionVector EFvel_d;

private:
    rl::mdl::Kinematic* kinematic;
};

class RobotFullBodyVelocityLimitconstraint: public JointVelocityConstraint
{
public:
    RobotFullBodyVelocityLimitconstraint(rl::mdl::Kinematic *kin);
    virtual int getNumConstraints();
    virtual Eigen::VectorXd calculateConstraintValue(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivative(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd getLowerBounds();
    virtual Eigen::VectorXd getUpperBounds();
    virtual void updateConstraintParams();

    Eigen::VectorXd q_curr;
private:
    rl::mdl::Kinematic* kinematic;
};

class RobotJointVelocityLimitconstraint: public JointVelocityConstraint
{
public:
    RobotJointVelocityLimitconstraint(rl::mdl::Kinematic *kin);
    virtual int getNumConstraints();
    virtual Eigen::VectorXd calculateConstraintValue(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivative(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd getLowerBounds();
    virtual Eigen::VectorXd getUpperBounds();
    virtual void updateConstraintParams();

    Eigen::VectorXd q_curr;

private:
    rl::mdl::Kinematic* kinematic;
};

class RobotDesiredPositionConstraint: public OperationalPositionConstraint
{
public:
    RobotDesiredPositionConstraint(rl::mdl::Kinematic *kin);
    virtual int getNumConstraints();
    virtual Eigen::VectorXd calculateConstraintValue(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivative(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd calculateConstraintValueAA(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivativeAA(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd getLowerBounds();
    virtual Eigen::VectorXd getUpperBounds();
    virtual void updateConstraintParams();

    rl::math::Transform EFpos_curr;
    rl::math::Transform EFpos_d;
    Eigen::VectorXd X_curr;
    Eigen::VectorXd X_d;
    int ef_id;
private:
    rl::mdl::Kinematic* kinematic;
};

class RobotDesiredVelocityConstraint: public OperationalVelocityConstraint
{
public:
    RobotDesiredVelocityConstraint(rl::mdl::Kinematic *kin);
    virtual int getNumConstraints();
    virtual Eigen::VectorXd calculateConstraintValue(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivative(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd getLowerBounds();
    virtual Eigen::VectorXd getUpperBounds();
    virtual void updateConstraintParams();

    rl::math::Transform EFpos_curr;
    rl::math::MotionVector EFvel_curr;
    rl::math::Transform EFpos_d;
    rl::math::MotionVector EFvel_d;
private:
    rl::mdl::Kinematic* kinematic;
};


class RobotDesiredJointPositionConstraint: public JointPositionConstraint
{
public:
    RobotDesiredJointPositionConstraint(rl::mdl::Kinematic *kin);
    virtual int getNumConstraints();
    virtual Eigen::VectorXd calculateConstraintValue(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivative(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd getLowerBounds();
    virtual Eigen::VectorXd getUpperBounds();
    virtual void updateConstraintParams();

    Eigen::VectorXd q_curr;
    Eigen::VectorXd q_d;
private:
    rl::mdl::Kinematic* kinematic;
};

class ManipulabilityConstraint: public JointPositionConstraint
{
public:
    ManipulabilityConstraint();
    ManipulabilityConstraint(rl::mdl::Kinematic *kin);
    virtual int getNumConstraints();
    virtual Eigen::VectorXd calculateConstraintValue(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivative(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd getLowerBounds();
    virtual Eigen::VectorXd getUpperBounds();
    virtual void updateConstraintParams();

    Eigen::MatrixXd tdots;
    Eigen::VectorXd q_curr;
private:
    rl::mdl::Kinematic* kinematic;

};

#include <rl/sg/solid/Scene.h>
#include <rl/sg/Body.h>
#include <rl/sg/DistanceScene.h>
#include <rl/sg/Model.h>
#include <rl/sg/Shape.h>
#include <rl/sg/SimpleScene.h>
#include <memory>

class CollisionConstraint: public Constraint
{
public:
    CollisionConstraint();
    CollisionConstraint(rl::mdl::Kinematic *kin, std::string fname);
    CollisionConstraint(rl::mdl::Kinematic *kin);
    virtual int getNumConstraints();
    virtual Eigen::VectorXd calculateConstraintValue(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivative(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd getLowerBounds();
    virtual Eigen::VectorXd getUpperBounds();
    virtual void updateConstraintParams();

    Eigen::MatrixXd getCollision(rl::sg::Body *body, std::vector<rl::math::Matrix> &J);

    std::shared_ptr< rl::sg::Scene > collisionScene;
    std::vector<Eigen::MatrixXd> tdots;
    Eigen::VectorXd q_curr;
    std::vector<rl::math::Matrix> J;
    std::vector<rl::sg::Body*> collision_bodies;
    Eigen::VectorXd min_link_distance;
private:
    rl::mdl::Kinematic* kinematic;
    bool collision_warning;

};

class CollisionConstraintPosition: public Constraint
{
public:
    CollisionConstraintPosition();
    CollisionConstraintPosition(rl::mdl::Kinematic *kin, std::string fname);
    CollisionConstraintPosition(rl::mdl::Kinematic *kin);
    virtual int getNumConstraints();
    virtual Eigen::VectorXd calculateConstraintValue(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivative(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd getLowerBounds();
    virtual Eigen::VectorXd getUpperBounds();
    virtual void updateConstraintParams();

    Eigen::MatrixXd getCollision(rl::sg::Body *body);

    std::shared_ptr< rl::sg::Scene > collisionScene;
    std::vector<Eigen::MatrixXd> points;
    Eigen::VectorXd q_curr;
    std::vector<rl::math::Matrix> J;
    std::vector<rl::sg::Body*> collision_bodies;
    double distance_threshold;
protected:
    rl::mdl::Kinematic* kinematic;    

};

class CollisionConstraintExact: public CollisionConstraintPosition
{
public:
    CollisionConstraintExact();
    CollisionConstraintExact(rl::mdl::Kinematic *kin, std::string fname);
    CollisionConstraintExact(rl::mdl::Kinematic *kin);
    virtual Eigen::VectorXd calculateConstraintValue(Eigen::VectorXd &vec);
};

class WorkspaceConfinementConstraint: public Constraint
{
public:
    WorkspaceConfinementConstraint();
    WorkspaceConfinementConstraint(rl::mdl::Kinematic *kin, std::string fname);
    virtual int getNumConstraints();
    virtual Eigen::VectorXd calculateConstraintValue(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivative(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd getLowerBounds();
    virtual Eigen::VectorXd getUpperBounds();
    virtual void updateConstraintParams();

    Eigen::MatrixXd getCollision(rl::sg::Body *body, std::vector<rl::math::Matrix> &J);

    std::shared_ptr< rl::sg::Scene > confinementScene;
    std::vector<Eigen::MatrixXd> tdots;
    Eigen::VectorXd q_curr;
    std::vector<rl::math::Matrix> J;
    std::vector<rl::sg::Body*> confinement_bodies;
private:
    rl::mdl::Kinematic* kinematic;

};

class RobotJointLimitConstraint: public JointPositionConstraint
{
public:
    RobotJointLimitConstraint(rl::mdl::Kinematic *kin);
    virtual int getNumConstraints();
    virtual Eigen::VectorXd calculateConstraintValue(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivative(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd getLowerBounds();
    virtual Eigen::VectorXd getUpperBounds();
    virtual void updateConstraintParams();
private:
    rl::mdl::Kinematic* kinematic;
};

class DualArmTransformConstraint: public OperationalPositionConstraint
{
public:
    DualArmTransformConstraint(rl::mdl::Kinematic *kin1, rl::mdl::Kinematic *kin2);
    virtual int getNumConstraints();
    virtual Eigen::VectorXd calculateConstraintValue(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivative(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd calculateConstraintValueAA(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivativeAA(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd getLowerBounds();
    virtual Eigen::VectorXd getUpperBounds();
    virtual void updateConstraintParams();

    rl::math::Transform T_12_d;
    rl::math::Transform EFpos1_curr;
    rl::math::Transform EFpos2_curr;
private:
    rl::mdl::Kinematic *kinematic1, *kinematic2;
};

class DualArmCenterDesiredPoseConstraint: public OperationalPositionConstraint
{
public:
    DualArmCenterDesiredPoseConstraint(rl::mdl::Kinematic *kin1, rl::mdl::Kinematic *kin2);
    virtual int getNumConstraints();
    virtual Eigen::VectorXd calculateConstraintValue(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivative(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd calculateConstraintValueAA(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivativeAA(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd getLowerBounds();
    virtual Eigen::VectorXd getUpperBounds();
    virtual void updateConstraintParams();

    rl::math::Transform T_center_d;
    rl::math::Transform EFpos1_curr;
    rl::math::Transform EFpos2_curr;
private:
    rl::mdl::Kinematic *kinematic1, *kinematic2;
};

#endif // ROBOTCONSTRAINTS_H
