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


#include "RobotConstraints.h"

RobotConstraints::RobotConstraints(rl::mdl::Kinematic* kin):
    Constraint(Configuration, Position, "robot_constraint"), kinematic(kin)
{

}

int RobotConstraints::getNumConstraints()
{
    return kinematic->getDof();
}

Eigen::VectorXd RobotConstraints::calculateConstraintValue(Eigen::VectorXd &vec)
{
    Eigen::Vector4d axisAngle = vec.segment(3,4);
    Eigen::Vector3d t = vec.segment(0,3);
    Eigen::VectorXd val;
    val.resize(kinematic->getDof());
    val = Eigen::VectorXd::Zero(kinematic->getDof());

    Eigen::Matrix3d R;
    Eigen::Matrix4d T;
    R = Eigen::AngleAxisd(axisAngle(3), axisAngle.segment(0,3));
    T.setIdentity();
    T.block(0,0,3,3) = R;
    T.block(0,3,3,1) = t;

    rl::math::Transform ef_pose=kinematic->getFrame(kinematic->getBodies()-1);
    rl::math::Transform x_robot;
    x_robot = T*(ef_pose.matrix());

    rl::math::Vector q(kinematic->getDof());
    q(0)=0.0f;
    q(1)=0.0f;
    q(2)=-1.57f;
    q(3)=0.0f;
    q(4)=1.57f;
    q(5)=-1.57f;
    kinematic->setPosition(q);
    kinematic->forwardPosition();
    if (!kinematic->calculateInversePosition(x_robot))
    {
        val = getUpperBounds()*1.1;
    }
    else
    {
        kinematic->getPosition(q);
        val = q;
    }
    //    std::cout << "q:" << val.transpose() << std::endl;
    return val;
}

Eigen::MatrixXd RobotConstraints::calculateConstraintDerivative(Eigen::VectorXd &vec)
{
    Eigen::Vector4d axisAngle = vec.segment(3,4);
    Eigen::Vector3d t = vec.segment(0,3);
    Eigen::MatrixXd val;
    val.resize(kinematic->getDof(),kinematic->getDof());
    val.setIdentity();
    val = Eigen::MatrixXd::Zero(kinematic->getDof(),7);
    double eps = 1e-10;
    for(std::size_t i = 0; i < 3; ++i)
    {
        Eigen::VectorXd Xplus, Xminus;
        Xplus.resize(7);Xminus.resize(7);
        Eigen::Vector3d eps_vector;
        eps_vector << 0,0,0;
        eps_vector(i) = eps;
        Eigen::Vector3d tplus, tminus;
        tplus = t+eps_vector;
        tminus = t-eps_vector;
        Xplus << tplus, axisAngle;
        Xminus << tminus, axisAngle;
        val.col(i) = (calculateConstraintValue(Xplus) - calculateConstraintValue(Xminus))
                / (2 * eps); // central differences
    }
    for(std::size_t i = 0; i < 4; ++i)
    {
        Eigen::Vector4d eps_vector = Eigen::VectorXd::Zero(4);
        eps_vector(i) = eps;
        Eigen::Vector4d AAminus, AAplus;
        AAminus = axisAngle-eps_vector;
        AAplus = axisAngle+eps_vector;
        AAminus.segment(0,3) /= AAminus.segment(0,3).norm();
        AAplus.segment(0,3) /= AAplus.segment(0,3).norm();
        if(AAplus(3) < 0) AAplus(3) += M_PI;
        if(AAplus(3) > M_PI) AAplus(3) -= M_PI;
        if(AAminus(3) < 0) AAminus(3) += M_PI;
        if(AAminus(3) > M_PI) AAminus(3) -= M_PI;
        Eigen::VectorXd Xplus, Xminus;
        Xplus.resize(7);Xminus.resize(7);
        Xplus << t, AAplus;
        Xminus << t, AAminus;
        val.col(i) = (calculateConstraintValue(Xplus) - calculateConstraintValue(Xminus))
                / (2 * eps); // central differences
    }
    //    std::cout << "derivative:" << val << std::endl;
    return val;
}

Eigen::VectorXd RobotConstraints::getLowerBounds()
{
    Eigen::VectorXd val;
    val.resize(kinematic->getDof());
    kinematic->getMinimum(val);
    return val;
}

Eigen::VectorXd RobotConstraints::getUpperBounds()
{
    Eigen::VectorXd val;
    val.resize(kinematic->getDof());
    kinematic->getMaximum(val);
    return val;
}

void RobotConstraints::updateConstraintParams()
{
}

//bool RobotConstraints::isConstraintSatisfied(Eigen::VectorXd vec)
//{
//    Eigen::VectorXd val = calculateConstraintValue(vec);
//    Eigen::VectorXd ub = getUpperBounds();
//    Eigen::VectorXd lb = getLowerBounds();
//    for(std::size_t constraint_id = 0;constraint_id < val.size();++constraint_id)
//    {
//        if(val(constraint_id) > ub(constraint_id) || val(constraint_id) < lb(constraint_id))
//            return false;
//    }
//    return true;
//}

Eigen::MatrixXd CollisionConstraint::getCollision(rl::sg::Body *body, std::vector<rl::math::Matrix> &J)
{
    collision_warning = false;
//    const clock_t begin_time = clock();

    Eigen::MatrixXd points;
    points.resize(kinematic->getBodies(), 6);
    rl::math::Real potentialRepulsiveScaling = 1e-2;
    rl::math::Real potentialRepulsiveThreshold = 0.05f;
    if (rl::sg::DistanceScene* distanceScene = dynamic_cast< rl::sg::DistanceScene* >(collisionScene.get()))
    {
        rl::math::Vector3 point1;
        rl::math::Vector3 point2;
        rl::math::Vector3 d;
        rl::math::Vector3 omega;
        rl::math::Vector3 v;
        rl::math::Vector6 tdot;
        min_link_distance.resize(kinematic->getBodies());
        for(std::size_t body_id = 0;body_id < kinematic->getBodies();++body_id)
        {
            collisionScene->getModel(0)->getBody(body_id)->setFrame(kinematic->getBody(body_id)->t);
            rl::math::Real distance = distanceScene->distance(
                        collisionScene->getModel(0)->getBody(body_id),
                        body,
                        point1,
                        point2
                        );
            //            std::cout << point1 << std::endl;
            //            std::cout << point2 << std::endl;
            //            std::cout << distance << std::endl;
            //            std::cout << (point1-point2).transpose() << std::endl;
            //            points.block(body_id, 0, 1, 3) = point1.transpose();
            //            points.block(body_id, 3, 1, 3) = point2.transpose();

//            std::cout << body_id << ": " << point1.transpose() << " --" << point2.transpose() << std::endl;
            d = point1 - kinematic->getBody(body_id)->t.translation();
            v = point1 - point2;
            min_link_distance(body_id) = distance;
            omega = d.cross(v);
            tdot.segment(0, 3) = v;
            tdot.segment(3, 3) = omega;
            if(distance > potentialRepulsiveThreshold)
                tdot *= 0;
            else
            {
                tdot *= potentialRepulsiveScaling * std::pow((1.0 / distance) - (1.0 / potentialRepulsiveThreshold), 2);
                collision_warning = true;
            }
            points.block(body_id, 0, 1, 6) = tdot.transpose();
            std::cout << "tdots:" << tdot.transpose() << std::endl;
            rl::math::Matrix J_i(kinematic->getOperationalDof() * 6, kinematic->getDof());
            rl::math::Transform T;
            T.setIdentity();
            T.translation().x() = d(0);
            T.translation().y() = d(1);
            T.translation().z() = d(2);
            calculateJacobian(J_i, body_id, T, kinematic);
            kinematic->setPosition(q_curr);
            kinematic->forwardPosition();
            J.push_back(J_i);
        }
    }
//    std::cout << points << std::endl;
    //    calculateJacobian(J);
    //    for(std::size_t body_id = 0;body_id < kinematic->getDof();++body_id)
    //        std::cout << (J[body_id]*(points.row(body_id).transpose())).transpose() << std::endl;

//    std::cout << "collision time: " << double( clock () - begin_time ) /  CLOCKS_PER_SEC << std::endl;
//    std::cout << min_link_distance.transpose() << std::endl;

    return points;
}


CollisionConstraint::CollisionConstraint():
    Constraint(Configuration, Position, "collision_constraint")
{

}

CollisionConstraint::CollisionConstraint(rl::mdl::Kinematic *kin, std::string fname):
    Constraint(Configuration, Position, "collision_constraint"),
    collisionScene(new rl::sg::solid::Scene()),
    kinematic(kin)

{
    collisionScene->load(fname, true, true);
}

CollisionConstraint::CollisionConstraint(rl::mdl::Kinematic *kin):
    Constraint(Configuration, Position, "collision_constraint"),
    kinematic(kin)
{
//    this->minimization_constraint = true;
}

int CollisionConstraint::getNumConstraints()
{
    return kinematic->getBodies()*collision_bodies.size();
}

Eigen::VectorXd CollisionConstraint::calculateConstraintValue(Eigen::VectorXd &vec)
{
    rl::math::Vector q_dot = vec - q_curr;
    Eigen::VectorXd val;
    val.resize(kinematic->getBodies()*collision_bodies.size());
    for(std::size_t body_id = 0;body_id < kinematic->getBodies();++body_id)
    {
        Eigen::VectorXd tdot = J[body_id]*(q_dot);
        for(std::size_t collision_body_id = 0;collision_body_id < collision_bodies.size();++collision_body_id)
        {
            val(kinematic->getBodies()*collision_body_id+body_id) = tdots[collision_body_id].row(body_id).dot(tdot);
        }
//        if(collision_warning)
//            std::cout << "tdot_iter:" << tdot.transpose() << std::endl;
    }
//    if(collision_warning)
//        std::cout << "constraint_value:" << val.transpose() << std::endl;
    return val;
}

Eigen::MatrixXd CollisionConstraint::calculateConstraintDerivative(Eigen::VectorXd &vec)
{
    Eigen::MatrixXd val;
    val.resize(kinematic->getBodies()*collision_bodies.size(),kinematic->getDof());
    val.setZero();
    for(std::size_t collision_body_id = 0;collision_body_id < collision_bodies.size();++collision_body_id)
    {
        for(std::size_t body_id = 0;body_id < kinematic->getBodies();++body_id)
        {
            for(std::size_t dof_id = 0;dof_id < kinematic->getDof();++dof_id)
            {
                val(kinematic->getBodies()*collision_body_id+body_id, dof_id) = (tdots[collision_body_id].row(body_id))*(J[body_id].col(dof_id));
            }
        }
    }
    return val;
}

Eigen::VectorXd CollisionConstraint::getLowerBounds()
{
    Eigen::VectorXd val;
    val.resize(kinematic->getBodies()*collision_bodies.size());
    for(std::size_t constraint_id = 0;constraint_id < kinematic->getBodies()*collision_bodies.size();++constraint_id)
    {
        if(collision_warning)
            val(constraint_id) = 1e-3;
        else
            val(constraint_id) = 0;
    }
    return val;
}

Eigen::VectorXd CollisionConstraint::getUpperBounds()
{
    Eigen::VectorXd val;
    val.resize(kinematic->getBodies()*collision_bodies.size());
    for(std::size_t constraint_id = 0;constraint_id < kinematic->getBodies()*collision_bodies.size();++constraint_id)
    {
        val(constraint_id) = 1e19;
    }
    return val;
}

void CollisionConstraint::updateConstraintParams()
{
    tdots.clear();
    for(std::size_t collision_body_id = 0;collision_body_id < collision_bodies.size();++collision_body_id)
    {
        tdots.push_back(getCollision(collision_bodies[collision_body_id], J));
    }
    q_curr.resize(kinematic->getDof());
    kinematic->getPosition(q_curr);
}

//bool CollisionConstraint::isConstraintSatisfied(Eigen::VectorXd vec)
//{
//    Eigen::VectorXd val = calculateConstraintValue(vec);
//    Eigen::VectorXd ub = getUpperBounds();
//    Eigen::VectorXd lb = getLowerBounds();
//    for(std::size_t constraint_id = 0;constraint_id < val.size();++constraint_id)
//    {
//        if(val(constraint_id) > ub(constraint_id) || val(constraint_id) < lb(constraint_id))
//            return false;
//    }
//    return true;
//}



WorkspaceConfinementConstraint::WorkspaceConfinementConstraint():
    Constraint(Configuration, Position, "workspace_confinement_constraint")
{

}

WorkspaceConfinementConstraint::WorkspaceConfinementConstraint(rl::mdl::Kinematic *kin, std::string fname):
    Constraint(Configuration, Position, "workspace_confinement_constraint"),
    confinementScene(new rl::sg::solid::Scene()),
    kinematic(kin)
{
    confinementScene->load(fname, true, true);
}

int WorkspaceConfinementConstraint::getNumConstraints()
{
    return kinematic->getBodies()*confinement_bodies.size();
}

Eigen::VectorXd WorkspaceConfinementConstraint::calculateConstraintValue(Eigen::VectorXd &vec)
{
    rl::math::Vector q_dot = vec - q_curr;
    Eigen::VectorXd val;
    val.resize(getNumConstraints());
    for(std::size_t body_id = 0;body_id < kinematic->getBodies();++body_id)
    {
        Eigen::VectorXd tdot = J[body_id]*(q_dot);
        for(std::size_t collision_body_id = 0;collision_body_id < confinement_bodies.size();++collision_body_id)
        {
            val(kinematic->getBodies()*collision_body_id+body_id) = tdots[collision_body_id].row(body_id)*tdot;
        }
    }
    return val;
}

Eigen::MatrixXd WorkspaceConfinementConstraint::calculateConstraintDerivative(Eigen::VectorXd &vec)
{
    Eigen::MatrixXd val;
    val.resize(getNumConstraints(),kinematic->getBodies());
    val.setZero();
    for(std::size_t collision_body_id = 0;collision_body_id < confinement_bodies.size();++collision_body_id)
    {
        for(std::size_t body_id = 0;body_id < kinematic->getBodies();++body_id)
        {
            for(std::size_t dof_id = 0;dof_id < kinematic->getDof();++dof_id)
            {
                val(kinematic->getBodies()*collision_body_id+body_id, dof_id) = (tdots[collision_body_id].row(body_id))*(J[body_id].col(dof_id));
            }
        }
    }
    return val;
}

Eigen::VectorXd WorkspaceConfinementConstraint::getLowerBounds()
{
    Eigen::VectorXd val;
    val.resize(getNumConstraints());
    for(std::size_t constraint_id = 0;constraint_id < val.size();++constraint_id)
    {
        val(constraint_id) = -1e19;
    }
    return val;
}

Eigen::VectorXd WorkspaceConfinementConstraint::getUpperBounds()
{
    Eigen::VectorXd val;
    val.resize(getNumConstraints());
    for(std::size_t constraint_id = 0;constraint_id < val.size();++constraint_id)
    {
        val(constraint_id) = 0;
    }
    return val;
}

void WorkspaceConfinementConstraint::updateConstraintParams()
{
    tdots.clear();
    for(std::size_t collision_body_id = 0;collision_body_id < confinement_bodies.size();++collision_body_id)
    {
        tdots.push_back(getCollision(confinement_bodies[collision_body_id], J));
    }
    q_curr.resize(kinematic->getDof());
    kinematic->getPosition(q_curr);
}

//bool WorkspaceConfinementConstraint::isConstraintSatisfied(Eigen::VectorXd vec)
//{
//    Eigen::VectorXd val = calculateConstraintValue(vec);
//    Eigen::VectorXd ub = getUpperBounds();
//    Eigen::VectorXd lb = getLowerBounds();
//    for(std::size_t constraint_id = 0;constraint_id < val.size();++constraint_id)
//    {
//        if(val(constraint_id) > ub(constraint_id) || val(constraint_id) < lb(constraint_id))
//            return false;
//    }
//    return true;
//}

Eigen::MatrixXd WorkspaceConfinementConstraint::getCollision(rl::sg::Body *body, std::vector<rl::math::Matrix> &J)
{
    const clock_t begin_time = clock();

    Eigen::MatrixXd points;
    points.resize(kinematic->getBodies(), 6);
    rl::math::Real potentialRepulsiveScaling = 0.005f;
    rl::math::Real potentialRepulsiveThreshold = 0.05f;
    if (rl::sg::DistanceScene* distanceScene = dynamic_cast< rl::sg::DistanceScene* >(confinementScene.get()))
    {
        rl::math::Vector3 point1;
        rl::math::Vector3 point2;
        rl::math::Vector3 d;
        rl::math::Vector3 omega;
        rl::math::Vector3 v;
        rl::math::Vector6 tdot;
        for(std::size_t body_id = 0;body_id < kinematic->getBodies();++body_id)
        {
            confinementScene->getModel(0)->getBody(body_id)->setFrame(kinematic->getBody(body_id)->t);
            rl::math::Real distance = distanceScene->distance(
                        confinementScene->getModel(0)->getBody(body_id),
                        body,
                        point1,
                        point2
                        );
            //            std::cout << point1 << std::endl;
            //            std::cout << point2 << std::endl;
            //            std::cout << distance << std::endl;
            //            std::cout << (point1-point2).transpose() << std::endl;
            //            points.block(body_id, 0, 1, 3) = point1.transpose();
            //            points.block(body_id, 3, 1, 3) = point2.transpose();

            d = point1 - kinematic->getBody(body_id)->t.translation();
            v = point1 - point2;
            omega = d.cross(v);
            tdot.segment(0, 3) = v;
            tdot.segment(3, 3) = omega;
            if(distance > potentialRepulsiveThreshold)
                tdot *= 0;
            else
                tdot *= 0.5 * potentialRepulsiveScaling * std::pow((1.0 / distance) - (1.0 / potentialRepulsiveThreshold), 2);
            points.block(body_id, 0, 1, 6) = tdot.transpose();
            rl::math::Matrix J_i(kinematic->getOperationalDof() * 6, kinematic->getDof());
            rl::math::Transform T;
            T.matrix().block(0,0,3,3).setIdentity();
            T.translation().x() = d(0);
            T.translation().y() = d(1);
            T.translation().z() = d(2);
            calculateJacobian(J_i, body_id, T, kinematic);
            J.push_back(J_i);

        }
    }
//    std::cout << points << std::endl;

    std::cout << "collision time: " << double( clock () - begin_time ) /  CLOCKS_PER_SEC << std::endl;

    return points;
}


RobotVelocityLimitconstraint::RobotVelocityLimitconstraint(rl::mdl::Kinematic *kin):
    OperationalVelocityConstraint("operational_velocity_constraint"),
    kinematic(kin)
{

}

int RobotVelocityLimitconstraint::getNumConstraints()
{
    return 1;
}

Eigen::VectorXd RobotVelocityLimitconstraint::calculateConstraintValue(Eigen::VectorXd &vec)
{
    Eigen::Matrix3d R;
    R = transformFromQuaternion(vec).block(0,0,3,3);
    Eigen::Vector3d t = vec.segment(0,3)+R*EFpos_curr.translation();
    Eigen::Vector3d tp_curr = EFvel_curr.linear();
    Eigen::Vector3d t_curr = EFpos_curr.translation();
    Eigen::VectorXd val;
    val.resize(1);
    val = (t-t_curr).transpose()*(t-t_curr);
    return val;
}

Eigen::MatrixXd RobotVelocityLimitconstraint::calculateConstraintDerivative(Eigen::VectorXd &vec)
{
    Eigen::Matrix3d R;
    R = transformFromQuaternion(vec).block(0,0,3,3);
    Eigen::Vector3d t = vec.segment(0,3)+R*EFpos_curr.translation();
    Eigen::Vector3d t_curr = EFpos_curr.translation();
    Eigen::VectorXd val;
    val.resize(1,7);
    val(0,0) = 2*(t(0)-t_curr(0));
    val(0,1) = 2*(t(1)-t_curr(1));
    val(0,2) = 2*(t(2)-t_curr(2));
    val(0,3) = 0;
    val(0,4) = 0;
    val(0,5) = 0;
    val(0,6) = 0;
    return val;
}

Eigen::VectorXd RobotVelocityLimitconstraint::getLowerBounds()
{
    Eigen::VectorXd val;
    val.resize(1);
    val(0) = 0;
    return val;
}

Eigen::VectorXd RobotVelocityLimitconstraint::getUpperBounds()
{
    Eigen::VectorXd val;
    val.resize(1);
    val(0) = 1e-6;
    return val;
}

void RobotVelocityLimitconstraint::updateConstraintParams()
{

}

//bool RobotVelocityLimitconstraint::isConstraintSatisfied(Eigen::VectorXd vec)
//{
//    Eigen::VectorXd val = calculateConstraintValue(vec);
//    Eigen::VectorXd ub = getUpperBounds();
//    Eigen::VectorXd lb = getLowerBounds();
//    for(std::size_t constraint_id = 0;constraint_id < val.size();++constraint_id)
//    {
//        if(val(constraint_id) > ub(constraint_id) || val(constraint_id) < lb(constraint_id))
//            return false;
//    }
//    return true;
//}


RobotJointVelocityLimitconstraint::RobotJointVelocityLimitconstraint(rl::mdl::Kinematic *kin):
    JointVelocityConstraint("robot_joint_velocity_limit_constraint"),
    kinematic(kin)
{

}

int RobotJointVelocityLimitconstraint::getNumConstraints()
{
    return 1;
}

Eigen::VectorXd RobotJointVelocityLimitconstraint::calculateConstraintValue(Eigen::VectorXd &vec)
{
    rl::math::Vector q_dot = vec - q_curr;
    Eigen::VectorXd val;
    val.resize(1);
    val(0) = q_dot.transpose()*q_dot;
    return val;
}

Eigen::MatrixXd RobotJointVelocityLimitconstraint::calculateConstraintDerivative(Eigen::VectorXd &vec)
{
    rl::math::Vector q_dot = vec - q_curr;
    Eigen::MatrixXd val;
    val.resize(1, kinematic->getDof());
    val.row(0) = 2*q_dot.transpose();
    return val;
}

Eigen::VectorXd RobotJointVelocityLimitconstraint::getLowerBounds()
{
    Eigen::VectorXd val;
    val.resize(1);
    val(0) = 0;
    return val;
}

Eigen::VectorXd RobotJointVelocityLimitconstraint::getUpperBounds()
{
    Eigen::VectorXd val;
    val.resize(1);
    val(0) = 1e-6;
    return val;
}

void RobotJointVelocityLimitconstraint::updateConstraintParams()
{

}

//bool RobotJointVelocityLimitconstraint::isConstraintSatisfied(Eigen::VectorXd vec)
//{
//    Eigen::VectorXd val = calculateConstraintValue(vec);
//    Eigen::VectorXd ub = getUpperBounds();
//    Eigen::VectorXd lb = getLowerBounds();
//    for(std::size_t constraint_id = 0;constraint_id < val.size();++constraint_id)
//    {
//        if(val(constraint_id) > ub(constraint_id) || val(constraint_id) < lb(constraint_id))
//            return false;
//    }
//    return true;
//}

RobotFullBodyVelocityLimitconstraint::RobotFullBodyVelocityLimitconstraint(rl::mdl::Kinematic *kin):
    JointVelocityConstraint("robot_fully_body_velocity_limit_constraint"),
    kinematic(kin)
{

}

int RobotFullBodyVelocityLimitconstraint::getNumConstraints()
{
    return kinematic->getDof();
}

Eigen::VectorXd RobotFullBodyVelocityLimitconstraint::calculateConstraintValue(Eigen::VectorXd &vec)
{
    rl::math::Vector q_dot = vec - q_curr;
    Eigen::VectorXd val;
    std::vector<rl::math::Matrix> J;
    calculateJacobian(J, kinematic);
    val.resize(getNumConstraints());
    for (std::size_t dof_id = 0; dof_id < kinematic->getDof(); ++dof_id)
    {
        rl::math::Vector t_dot = J[dof_id]*q_dot;
        val(dof_id) = t_dot.transpose()*t_dot;
    }
    return val;
}

Eigen::MatrixXd RobotFullBodyVelocityLimitconstraint::calculateConstraintDerivative(Eigen::VectorXd &vec)
{
    rl::math::Vector q_dot = vec - q_curr;
    Eigen::MatrixXd val;
    std::vector<rl::math::Matrix> J;
    calculateJacobian(J, kinematic);
    val.resize(getNumConstraints(), kinematic->getDof());
    for (std::size_t constraint_id = 0; constraint_id < getNumConstraints(); ++constraint_id)
    {
        for (std::size_t dof_id = 0; dof_id < kinematic->getDof(); ++dof_id)
        {
            rl::math::Vector t_dot = J[constraint_id]*q_dot;
            val(constraint_id, dof_id) = 2*t_dot.transpose()*(J[constraint_id].col(dof_id));
        }
    }
    return val;
}

Eigen::VectorXd RobotFullBodyVelocityLimitconstraint::getLowerBounds()
{
    Eigen::VectorXd val;
    val.resize(getNumConstraints());
    for (std::size_t constraint_id = 0; constraint_id < getNumConstraints(); ++constraint_id)
    {
        val(constraint_id) = 0;
    }
    return val;
}

Eigen::VectorXd RobotFullBodyVelocityLimitconstraint::getUpperBounds()
{
    Eigen::VectorXd val;
    val.resize(getNumConstraints());
    for (std::size_t constraint_id = 0; constraint_id < getNumConstraints(); ++constraint_id)
    {
        val(constraint_id) = 1e-4;
    }
    return val;
}

void RobotFullBodyVelocityLimitconstraint::updateConstraintParams()
{

}

//bool RobotFullBodyVelocityLimitconstraint::isConstraintSatisfied(Eigen::VectorXd vec)
//{
//    Eigen::VectorXd val = calculateConstraintValue(vec);
//    Eigen::VectorXd ub = getUpperBounds();
//    Eigen::VectorXd lb = getLowerBounds();
//    for(std::size_t constraint_id = 0;constraint_id < val.size();++constraint_id)
//    {
//        if(val(constraint_id) > ub(constraint_id) || val(constraint_id) < lb(constraint_id))
//            return false;
//    }
//    return true;
//}

ManipulabilityConstraint::ManipulabilityConstraint():
    JointPositionConstraint("manipulability_constraint")
{

}

ManipulabilityConstraint::ManipulabilityConstraint(rl::mdl::Kinematic *kin):
    JointPositionConstraint("manipulability_constraint"),
    kinematic(kin)
{
    this->minimization_constraint=true;
}

int ManipulabilityConstraint::getNumConstraints()
{
    return 1;
}

Eigen::VectorXd ManipulabilityConstraint::calculateConstraintValue(Eigen::VectorXd &vec)
{

    Eigen::VectorXd val;
    val.resize(1);
    kinematic->setPosition(vec);
    kinematic->calculateJacobian();
    val(0) = 1.0/(1.0+kinematic->calculateManipulabilityMeasure());
    val(0) *= val(0);
//    std::cout << kinematic->calculateManipulabilityMeasure() << std::endl;
    return val;
}

Eigen::MatrixXd ManipulabilityConstraint::calculateConstraintDerivative(Eigen::VectorXd &vec)
{
    Eigen::MatrixXd val;
    val.resize(1,kinematic->getDof());

    Eigen::VectorXd q_minus, q_plus;
    q_minus.resize(vec.size());
    q_plus.resize(vec.size());
    double eps = 1e-8;
    for(std::size_t q_id = 0;q_id < vec.size();++q_id)
    {
        q_minus = vec;
        q_plus = vec;
        q_minus(q_id) -= eps;
        q_plus(q_id) += eps;
        double val_minus = calculateConstraintValue(q_minus)(0);
        double val_plus = calculateConstraintValue(q_plus)(0);
        val(0,q_id) = (val_plus-val_minus)/(2.0*eps);
    }

    return val;
}

Eigen::VectorXd ManipulabilityConstraint::getLowerBounds()
{
    Eigen::VectorXd val;
    val.resize(1);
    val(0) = 0;
    return val;
}

Eigen::VectorXd ManipulabilityConstraint::getUpperBounds()
{
    Eigen::VectorXd val;
    val.resize(1);
    val(0) = 1e19;
    return val;
}

void ManipulabilityConstraint::updateConstraintParams()
{

}

RobotDesiredPositionConstraint::RobotDesiredPositionConstraint(rl::mdl::Kinematic *kin):
    OperationalPositionConstraint("desired_position_constraint"),
    kinematic(kin),
    ef_id(0)
{
    this->minimization_constraint = true;
}

int RobotDesiredPositionConstraint::getNumConstraints()
{
    return 1;
}

Eigen::VectorXd RobotDesiredPositionConstraint::calculateConstraintValue(Eigen::VectorXd &vec)
{
    Eigen::VectorXd X_iter = applyTransformQuaternion(vec.segment(ef_id*7,7), X_curr);
    Eigen::VectorXd val;
    val.resize(getNumConstraints());

    rl::math::Transform t_iter, t_d;
    t_iter.matrix() = transformFromQuaternion(X_iter);
    t_d.matrix() = transformFromQuaternion(X_d);
    rl::math::Vector delta_x(6);
    rl::math::transform::toDelta(t_iter, t_d, delta_x);
//    delta_x = t_iter.toDelta(t_d);
    if (delta_x.segment(0, 3).cwiseAbs().maxCoeff() < 1e-6)
    {
        delta_x.segment(0, 3).setZero();
    }
    if (delta_x.segment(3, 3).cwiseAbs().maxCoeff() < 1e-6)
    {
        delta_x.segment(3, 3).setZero();
    }

//    val = delta_x;

    val(0) = delta_x.dot(delta_x);
//    val(0) = delta_x.norm();

//    double dist = rl::math::transform::distance<rl::math::Transform, rl::math::Transform, double> (t_iter, t_d);
//    val(0) = dist*dist;
//    double distance_rotation = distanceRotations(X_iter.segment(3,4), X_d.segment(3,4));
//    val(0) = (X_iter.segment(0,3)-X_d.segment(0,3)).transpose()*(X_iter.segment(0,3)-X_d.segment(0,3));
//    val(0) += distance_rotation*distance_rotation;
    return val;
}


Eigen::VectorXd RobotDesiredPositionConstraint::calculateConstraintValueAA(Eigen::VectorXd &vec)
{
    Eigen::VectorXd X_iter = applyTransform(vec.segment(ef_id*7,7), X_curr);
    Eigen::VectorXd val;
    val.resize(getNumConstraints());

    rl::math::Transform t_iter, t_d;
    t_iter.matrix() = transformFromVector(X_iter);
    t_d.matrix() = transformFromVector(X_d);
    rl::math::Vector delta_x(6);
    rl::math::transform::toDelta(t_iter, t_d, delta_x);
//    delta_x = t_iter.toDelta(t_d);
    if (delta_x.segment(0, 3).cwiseAbs().maxCoeff() < 1e-6)
    {
        delta_x.segment(0, 3).setZero();
    }
    if (delta_x.segment(3, 3).cwiseAbs().maxCoeff() < 1e-6)
    {
        delta_x.segment(3, 3).setZero();
    }

//    val = delta_x;

    val(0) = delta_x.dot(delta_x);

//    double dist = rl::math::transform::distance<rl::math::Transform, rl::math::Transform, double> (t_iter, t_d);
//    val(0) = dist*dist;
//    double distance_rotation = distanceRotations(X_iter.segment(3,4), X_d.segment(3,4));
//    val(0) = (X_iter.segment(0,3)-X_d.segment(0,3)).transpose()*(X_iter.segment(0,3)-X_d.segment(0,3));
//    val(0) += distance_rotation*distance_rotation;
    return val;
}

Eigen::MatrixXd RobotDesiredPositionConstraint::calculateConstraintDerivativeAA(Eigen::VectorXd &vec)
{

}

Eigen::MatrixXd RobotDesiredPositionConstraint::calculateConstraintDerivative(Eigen::VectorXd &vec)
{
//    Eigen::VectorXd X_iter = applyTransform(vec, X_curr);
//    Eigen::Vector4d distance_rotation_d = distanceRotationsDerivative(X_iter.segment(3,4), X_d.segment(3,4));
//    double distance_rotation = distanceRotations(X_iter.segment(3,4), X_d.segment(3,4));

//    Eigen::MatrixXd val;
//    val.resize(1,7);
//    val(0,0) = 2*(X_iter(0)-X_d(0));
//    val(0,1) = 2*(X_iter(1)-X_d(1));
//    val(0,2) = 2*(X_iter(2)-X_d(2));
//    //    val(0,3) = 2*(X_iter.segment(0,3)-X_d.segment(0,3)).transpose()*getdRx(vec.segment(3,4))*X_iter.segment(0,3)+2*distance_rotation*distance_rotation_d(0);
//    //    val(0,4) = 2*(X_iter.segment(0,3)-X_d.segment(0,3)).transpose()*getdRy(vec.segment(3,4))*X_iter.segment(0,3)+2*distance_rotation*distance_rotation_d(1);
//    //    val(0,5) = 2*(X_iter.segment(0,3)-X_d.segment(0,3)).transpose()*getdRz(vec.segment(3,4))*X_iter.segment(0,3)+2*distance_rotation*distance_rotation_d(2);
//    //    val(0,6) = 2*(X_iter.segment(0,3)-X_d.segment(0,3)).transpose()*getdRtheta(vec.segment(3,4))*X_iter.segment(0,3)+2*distance_rotation*distance_rotation_d(3);
//    val(0,3) = 2*distance_rotation*distance_rotation_d(0);
//    val(0,4) = 2*distance_rotation*distance_rotation_d(1);
//    val(0,5) = 2*distance_rotation*distance_rotation_d(2);
//    val(0,6) = 2*distance_rotation*distance_rotation_d(3);

    Eigen::MatrixXd val;
    val.resize(getNumConstraints(),7);
    Eigen::VectorXd dX_minus, dX_plus;
    dX_minus.resize(7);
    dX_plus.resize(7);
    double eps = 1e-7;
    for(std::size_t dx_id = 0;dx_id < 7;++dx_id)
    {
        dX_minus = vec;
        dX_plus = vec;
        dX_minus(dx_id) -= eps;
        dX_plus(dx_id) += eps;
        dX_minus.segment(3,4).normalize();
        dX_plus.segment(3,4).normalize();
        Eigen::VectorXd val_minus = calculateConstraintValue(dX_minus);
        Eigen::VectorXd val_plus = calculateConstraintValue(dX_plus);
        val.col(dx_id) = (val_plus-val_minus)/(2.0*eps);
    }

    return val;
}

Eigen::VectorXd RobotDesiredPositionConstraint::getLowerBounds()
{
    Eigen::VectorXd val;
    val.resize(getNumConstraints());
    val << 0;//,0,0,0,0,0;
    return val;
}

Eigen::VectorXd RobotDesiredPositionConstraint::getUpperBounds()
{
    Eigen::VectorXd val;
    val.resize(getNumConstraints());
    val << 1e-6;//,1e-6,1e-6,1e-6,1e-6,1e-6;
    return val;
}

void RobotDesiredPositionConstraint::updateConstraintParams()
{
    X_curr = quaternionFromTransform(EFpos_curr.matrix());
    X_d = quaternionFromTransform(EFpos_d.matrix());
}

RobotDesiredJointPositionConstraint::RobotDesiredJointPositionConstraint(rl::mdl::Kinematic *kin):
    JointPositionConstraint("desired_joint_position_constraint"),
    kinematic(kin)
{
    this->minimization_constraint = true;
}

int RobotDesiredJointPositionConstraint::getNumConstraints()
{
    return 1;
}

Eigen::VectorXd RobotDesiredJointPositionConstraint::calculateConstraintValue(Eigen::VectorXd &vec)
{
    Eigen::VectorXd val;
    val.resize(1);
    val(0) = (vec-q_d).transpose()*(vec-q_d);
    return val;
}

Eigen::MatrixXd RobotDesiredJointPositionConstraint::calculateConstraintDerivative(Eigen::VectorXd &vec)
{
    Eigen::MatrixXd val;
    val.resize(1,q_d.size());
    val.row(0) = 2*(vec-q_d).transpose();
    return val;
}

Eigen::VectorXd RobotDesiredJointPositionConstraint::getLowerBounds()
{
    Eigen::VectorXd val;
    val.resize(1);
    val(0) = 0;
    return val;
}

Eigen::VectorXd RobotDesiredJointPositionConstraint::getUpperBounds()
{
    Eigen::VectorXd val;
    val.resize(1);
    val(0) = 1e-6;
    return val;
}

void RobotDesiredJointPositionConstraint::updateConstraintParams()
{

}

RobotDesiredVelocityConstraint::RobotDesiredVelocityConstraint(rl::mdl::Kinematic *kin):
    OperationalVelocityConstraint("operational_velocity_constraint"),
    kinematic(kin)
{
//    this->minimization_constraint = true;
}

int RobotDesiredVelocityConstraint::getNumConstraints()
{
    return 1;
}

Eigen::VectorXd RobotDesiredVelocityConstraint::calculateConstraintValue(Eigen::VectorXd &vec)
{
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(vec(6), vec.segment(3,3));
    Eigen::Vector3d t = vec.segment(0,3)+R*EFpos_curr.translation();
    Eigen::Vector3d t_curr = EFpos_curr.translation();
    Eigen::Vector3d tp_curr = t-t_curr;
    Eigen::VectorXd val;
//    std::cout << "tp_curr: " << tp_curr.transpose() << std::endl;
//    std::cout << "tp_d: " << EFvel_d.linear().transpose() << std::endl;
    val.resize(1);
    val(0) = (tp_curr-EFvel_d.linear()).transpose()*(tp_curr-EFvel_d.linear());
//    std::cout << val << std::endl;
    return val;
}

Eigen::MatrixXd RobotDesiredVelocityConstraint::calculateConstraintDerivative(Eigen::VectorXd &vec)
{
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(vec(6), vec.segment(3,3));
    Eigen::Vector3d t = vec.segment(0,3)+R*EFpos_curr.translation();
    Eigen::Vector3d t_curr = EFpos_curr.translation();
    Eigen::Vector3d tp_curr = t-t_curr;
    Eigen::VectorXd val;
    val.resize(1,7);
    val(0,0) = 2*(tp_curr-EFvel_d.linear())(0);
    val(0,1) = 2*(tp_curr-EFvel_d.linear())(1);
    val(0,2) = 2*(tp_curr-EFvel_d.linear())(2);
    val(0,3) = 2*((tp_curr-EFvel_d.linear()).transpose())*getdRx(vec.segment(3,4))*EFpos_curr.translation();
    val(0,4) = 2*((tp_curr-EFvel_d.linear()).transpose())*getdRy(vec.segment(3,4))*EFpos_curr.translation();
    val(0,5) = 2*((tp_curr-EFvel_d.linear()).transpose())*getdRz(vec.segment(3,4))*EFpos_curr.translation();
    val(0,6) = 2*((tp_curr-EFvel_d.linear()).transpose())*getdRtheta(vec.segment(3,4))*EFpos_curr.translation();
    return val;
}

Eigen::VectorXd RobotDesiredVelocityConstraint::getLowerBounds()
{
    Eigen::VectorXd val;
    val.resize(1);
    val(0) = 0;
    return val;
}

Eigen::VectorXd RobotDesiredVelocityConstraint::getUpperBounds()
{
    Eigen::VectorXd val;
    val.resize(1);
    val(0) = 0;
    return val;
}

void RobotDesiredVelocityConstraint::updateConstraintParams()
{

}

//bool RobotDesiredVelocityConstraint::isConstraintSatisfied(Eigen::VectorXd vec)
//{

//}


RobotJointLimitConstraint::RobotJointLimitConstraint(rl::mdl::Kinematic *kin):
    JointPositionConstraint("joint_limits_constraint"),
    kinematic(kin)
{

}

int RobotJointLimitConstraint::getNumConstraints()
{
    return kinematic->getDof();
}

Eigen::VectorXd RobotJointLimitConstraint::calculateConstraintValue(Eigen::VectorXd &vec)
{
    Eigen::VectorXd val;
    val = vec;
    return val;
}

Eigen::MatrixXd RobotJointLimitConstraint::calculateConstraintDerivative(Eigen::VectorXd &vec)
{
    Eigen::MatrixXd val;
    val.resize(vec.size(), vec.size());
    val.setIdentity();
    return val;
}

Eigen::VectorXd RobotJointLimitConstraint::getLowerBounds()
{
    Eigen::VectorXd lb;
    lb.resize(kinematic->getDof());
    kinematic->getMinimum(lb);
    return lb;
}

Eigen::VectorXd RobotJointLimitConstraint::getUpperBounds()
{
    Eigen::VectorXd ub;
    ub.resize(kinematic->getDof());
    kinematic->getMaximum(ub);
    return ub;
}

void RobotJointLimitConstraint::updateConstraintParams()
{

}


DualArmTransformConstraint::DualArmTransformConstraint(rl::mdl::Kinematic *kin1, rl::mdl::Kinematic *kin2):
    OperationalPositionConstraint("dual_arm_transform_constraint"),
    kinematic1(kin1),
    kinematic2(kin2)
{
    T_12_d.setIdentity();
}

int DualArmTransformConstraint::getNumConstraints()
{
    return 1;
}

Eigen::VectorXd DualArmTransformConstraint::calculateConstraintValue(Eigen::VectorXd &vec)
{
    rl::math::Transform t1_offset;
    t1_offset.matrix() = transformFromQuaternion(vec.segment(0,7));
    rl::math::Transform t2_offset;
    t2_offset.matrix() = transformFromQuaternion(vec.segment(7,7));

    rl::math::Transform t1_d, t1;
    rl::math::Transform t2_d, t2;
    t1 = t1_offset*EFpos1_curr;
    t2 = t2_offset*EFpos2_curr;

//    Eigen::VectorXd val;
//    val.resize(2);

//    t1_d = t2*T_12_d;
//    rl::math::Vector delta_x_1(6);
//    rl::math::transform::toDelta(t1, t1_d, delta_x_1);
//    val(0) = std::sqrt(delta_x_1.dot(delta_x_1));

//    t2_d = t1*T_12_d.inverse();
//    rl::math::Vector delta_x_2(6);
//    rl::math::transform::toDelta(t2, t2_d, delta_x_2);
//    val(1) = std::sqrt(delta_x_2.dot(delta_x_2));


    Eigen::VectorXd val;
    val.resize(1);

    rl::math::Transform t12;
    t12 = t2.inverse()*t1;

    rl::math::Vector delta_x(6);
    rl::math::transform::toDelta(t12, T_12_d, delta_x);
//    delta_x = t12.toDelta(T_12_d);
    val(0) = std::sqrt(delta_x.dot(delta_x));


//    Eigen::VectorXd X1_curr = vectorFromTransform(EFpos1_curr.matrix());
//    Eigen::VectorXd X2_curr = vectorFromTransform(EFpos2_curr.matrix());

//    Eigen::VectorXd X1 = applyTransform(vec.segment(0,7), X1_curr);
//    Eigen::VectorXd X2 = applyTransform(vec.segment(7,7), X2_curr);

//    Eigen::VectorXd X_12_d = vectorFromTransform(T_12_d.matrix());
//    Eigen::VectorXd X_21_d = vectorFromTransform(T_12_d.inverse().matrix());

//////    std::cout << "T_12:" << vectorFromTransform(transformFromVector(X1).inverse()*(transformFromVector(X2))).transpose() << std::endl;
//////    std::cout << "X_12:" << X_12_d.transpose() << std::endl;

//    Eigen::VectorXd X1_d = applyTransform(X_12_d, X2);
//    Eigen::VectorXd X2_d = applyTransform(X_21_d, X1);

//    Eigen::VectorXd val;
//    val.resize(2);
//    rl::math::Transform t1_d, t1;
//    t1_d.matrix() = transformFromVector(X1_d);
//    t1.matrix() = transformFromVector(X1);
//    rl::math::Vector delta_x_1(6);
//    rl::math::transform::toDelta(t1, t1_d, delta_x_1);
//    val(0) = delta_x_1.dot(delta_x_1);

//    rl::math::Transform t2_d, t2;
//    t2_d.matrix() = transformFromVector(X2_d);
//    t2.matrix() = transformFromVector(X2);
//    rl::math::Vector delta_x_2(6);
//    rl::math::transform::toDelta(t2, t2_d, delta_x_2);
//    val(1) = delta_x_2.dot(delta_x_2);


//    Eigen::VectorXd val;
//    val.resize(2);
////    double distance_rotation1 = distanceRotations(X1.segment(3,4), X1_d.segment(3,4));
//    val(0) = (X1.segment(0,3)-X1_d.segment(0,3)).transpose()*(X1.segment(0,3)-X1_d.segment(0,3));
////    val(0) += distance_rotation1*distance_rotation1;
////    double distance_rotation2 = distanceRotations(X2.segment(3,4), X2_d.segment(3,4));
//    val(1) = (X2.segment(0,3)-X2_d.segment(0,3)).transpose()*(X2.segment(0,3)-X2_d.segment(0,3));
////    val(1) += distance_rotation2*distance_rotation2;
////    std::cout << "constraint val:" << val.transpose() << std::endl;
    return val;
}

Eigen::MatrixXd DualArmTransformConstraint::calculateConstraintDerivative(Eigen::VectorXd &vec)
{
    Eigen::MatrixXd val;
    val.resize(2,14);
    Eigen::VectorXd dX_minus, dX_plus;
    dX_minus.resize(14);
    dX_plus.resize(14);
    double eps = 1e-6;
    for(std::size_t dx_id = 0;dx_id < 14;++dx_id)
    {
        dX_minus = vec;
        dX_plus = vec;
        dX_minus(dx_id) -= eps;
        dX_plus(dx_id) += eps;
        dX_minus.segment(3,3).normalize();
        dX_plus.segment(3,3).normalize();
        dX_minus.segment(10,3).normalize();
        dX_plus.segment(10,3).normalize();
        double val_minus = calculateConstraintValue(dX_minus)(0);
        double val_plus = calculateConstraintValue(dX_plus)(0);
        val(0,dx_id) = (val_plus-val_minus)/(2.0*eps);
        val_minus = calculateConstraintValue(dX_minus)(1);
        val_plus = calculateConstraintValue(dX_plus)(1);
        val(1,dx_id) = (val_plus-val_minus)/(2.0*eps);
    }

    return val;
}

Eigen::VectorXd DualArmTransformConstraint::calculateConstraintValueAA(Eigen::VectorXd &vec)
{
    rl::math::Transform t1_offset;
    t1_offset.matrix() = transformFromVector(vec.segment(0,7));
    rl::math::Transform t2_offset;
    t2_offset.matrix() = transformFromVector(vec.segment(7,7));

    rl::math::Transform t1_d, t1;
    rl::math::Transform t2_d, t2;
    t1 = t1_offset*EFpos1_curr;
    t2 = t2_offset*EFpos2_curr;

//    Eigen::VectorXd val;
//    val.resize(2);

//    t1_d = t2*T_12_d;
//    rl::math::Vector delta_x_1(6);
//    rl::math::transform::toDelta(t1, t1_d, delta_x_1);
//    val(0) = std::sqrt(delta_x_1.dot(delta_x_1));

//    t2_d = t1*T_12_d.inverse();
//    rl::math::Vector delta_x_2(6);
//    rl::math::transform::toDelta(t2, t2_d, delta_x_2);
//    val(1) = std::sqrt(delta_x_2.dot(delta_x_2));


    Eigen::VectorXd val;
    val.resize(1);

    rl::math::Transform t12;
    t12 = t2.inverse()*t1;

    rl::math::Vector delta_x(6);
    rl::math::transform::toDelta(t12, T_12_d, delta_x);
//    delta_x = t12.toDelta(T_12_d);
    val(0) = std::sqrt(delta_x.dot(delta_x));


//    Eigen::VectorXd X1_curr = vectorFromTransform(EFpos1_curr.matrix());
//    Eigen::VectorXd X2_curr = vectorFromTransform(EFpos2_curr.matrix());

//    Eigen::VectorXd X1 = applyTransform(vec.segment(0,7), X1_curr);
//    Eigen::VectorXd X2 = applyTransform(vec.segment(7,7), X2_curr);

//    Eigen::VectorXd X_12_d = vectorFromTransform(T_12_d.matrix());
//    Eigen::VectorXd X_21_d = vectorFromTransform(T_12_d.inverse().matrix());

//////    std::cout << "T_12:" << vectorFromTransform(transformFromVector(X1).inverse()*(transformFromVector(X2))).transpose() << std::endl;
//////    std::cout << "X_12:" << X_12_d.transpose() << std::endl;

//    Eigen::VectorXd X1_d = applyTransform(X_12_d, X2);
//    Eigen::VectorXd X2_d = applyTransform(X_21_d, X1);

//    Eigen::VectorXd val;
//    val.resize(2);
//    rl::math::Transform t1_d, t1;
//    t1_d.matrix() = transformFromVector(X1_d);
//    t1.matrix() = transformFromVector(X1);
//    rl::math::Vector delta_x_1(6);
//    rl::math::transform::toDelta(t1, t1_d, delta_x_1);
//    val(0) = delta_x_1.dot(delta_x_1);

//    rl::math::Transform t2_d, t2;
//    t2_d.matrix() = transformFromVector(X2_d);
//    t2.matrix() = transformFromVector(X2);
//    rl::math::Vector delta_x_2(6);
//    rl::math::transform::toDelta(t2, t2_d, delta_x_2);
//    val(1) = delta_x_2.dot(delta_x_2);


//    Eigen::VectorXd val;
//    val.resize(2);
////    double distance_rotation1 = distanceRotations(X1.segment(3,4), X1_d.segment(3,4));
//    val(0) = (X1.segment(0,3)-X1_d.segment(0,3)).transpose()*(X1.segment(0,3)-X1_d.segment(0,3));
////    val(0) += distance_rotation1*distance_rotation1;
////    double distance_rotation2 = distanceRotations(X2.segment(3,4), X2_d.segment(3,4));
//    val(1) = (X2.segment(0,3)-X2_d.segment(0,3)).transpose()*(X2.segment(0,3)-X2_d.segment(0,3));
////    val(1) += distance_rotation2*distance_rotation2;
////    std::cout << "constraint val:" << val.transpose() << std::endl;
    return val;

}

Eigen::MatrixXd DualArmTransformConstraint::calculateConstraintDerivativeAA(Eigen::VectorXd &vec)
{

}

Eigen::VectorXd DualArmTransformConstraint::getLowerBounds()
{
    Eigen::VectorXd lb;
    lb.resize(1);
    lb << 0;
    return lb;
}

Eigen::VectorXd DualArmTransformConstraint::getUpperBounds()
{
    Eigen::VectorXd ub;
    ub.resize(1);
    ub << 1e-6;
    return ub;
}

void DualArmTransformConstraint::updateConstraintParams()
{

}

DualArmCenterDesiredPoseConstraint::DualArmCenterDesiredPoseConstraint(rl::mdl::Kinematic *kin1, rl::mdl::Kinematic *kin2):
    OperationalPositionConstraint("dual_arm_center_desired_pose_constraint"),
    kinematic1(kin1),
    kinematic2(kin2)
{
    T_center_d.setIdentity();
}

int DualArmCenterDesiredPoseConstraint::getNumConstraints()
{
    return 1;
}

Eigen::VectorXd DualArmCenterDesiredPoseConstraint::calculateConstraintValue(Eigen::VectorXd &vec)
{
//    Eigen::VectorXd X1_curr = vectorFromTransform(EFpos1_curr.matrix());
//    Eigen::VectorXd X2_curr = vectorFromTransform(EFpos2_curr.matrix());

//    Eigen::VectorXd X1 = applyTransform(vec.segment(0,7), X1_curr);
//    Eigen::VectorXd X2 = applyTransform(vec.segment(7,7), X2_curr);

    Eigen::VectorXd X_center_d = quaternionFromTransform(T_center_d.matrix());

//    Eigen::VectorXd X_center = X1;
//    X_center.segment(0,3) += X2.segment(0,3);
//    X_center.segment(0,3) /= 2;

    rl::math::Transform t1_offset;
    t1_offset.matrix() = transformFromQuaternion(vec.segment(0,7));
    rl::math::Transform t2_offset;
    t2_offset.matrix() = transformFromQuaternion(vec.segment(7,7));

    rl::math::Transform t1;
    rl::math::Transform t2;
    t1 = t1_offset*EFpos1_curr;
    t2 = t2_offset*EFpos2_curr;

    Eigen::VectorXd X_center;
    X_center.resize(7);
    X_center.segment(0,3) = (t1.translation()+t2.translation())/2;

    Eigen::VectorXd val;
    val.resize(1);
//    double distance_rotation1 = distanceRotations(X_center.segment(3,4), X_center_d.segment(3,4));
    val(0) = (X_center.segment(0,3)-X_center_d.segment(0,3)).transpose()*(X_center.segment(0,3)-X_center_d.segment(0,3));
//    val = (X_center.segment(0,3)-X_center_d.segment(0,3));
//    val(0) = std::sqrt(val(0));
//    val(0) += distance_rotation1*distance_rotation1;
//    std::cout << "constraint val:" << val.transpose() << std::endl;
//    std::cout << "X_center:" << X_center.transpose() << std::endl;
    return val;
}

Eigen::MatrixXd DualArmCenterDesiredPoseConstraint::calculateConstraintDerivative(Eigen::VectorXd &vec)
{
    Eigen::MatrixXd val;
    val.resize(1,14);
    Eigen::VectorXd dX_minus, dX_plus;
    dX_minus.resize(14);
    dX_plus.resize(14);
    double eps = 1e-8;
    for(std::size_t dx_id = 0;dx_id < 14;++dx_id)
    {
        dX_minus = vec;
        dX_plus = vec;
        dX_minus(dx_id) -= eps;
        dX_plus(dx_id) += eps;
        dX_minus.segment(3,3).normalize();
        dX_plus.segment(3,3).normalize();
        dX_minus.segment(10,3).normalize();
        dX_plus.segment(10,3).normalize();
        Eigen::VectorXd val_minus = calculateConstraintValue(dX_minus);
        Eigen::VectorXd val_plus = calculateConstraintValue(dX_plus);
        val.col(dx_id) = (val_plus-val_minus)/(2.0*eps);
    }

    return val;
}

Eigen::VectorXd DualArmCenterDesiredPoseConstraint::calculateConstraintValueAA(Eigen::VectorXd &vec)
{
    //    Eigen::VectorXd X1_curr = vectorFromTransform(EFpos1_curr.matrix());
    //    Eigen::VectorXd X2_curr = vectorFromTransform(EFpos2_curr.matrix());

    //    Eigen::VectorXd X1 = applyTransform(vec.segment(0,7), X1_curr);
    //    Eigen::VectorXd X2 = applyTransform(vec.segment(7,7), X2_curr);

        Eigen::VectorXd X_center_d = vectorFromTransform(T_center_d.matrix());

    //    Eigen::VectorXd X_center = X1;
    //    X_center.segment(0,3) += X2.segment(0,3);
    //    X_center.segment(0,3) /= 2;

        rl::math::Transform t1_offset;
        t1_offset.matrix() = transformFromVector(vec.segment(0,7));
        rl::math::Transform t2_offset;
        t2_offset.matrix() = transformFromVector(vec.segment(7,7));

        rl::math::Transform t1;
        rl::math::Transform t2;
        t1 = t1_offset*EFpos1_curr;
        t2 = t2_offset*EFpos2_curr;

        Eigen::VectorXd X_center;
        X_center.resize(7);
        X_center.segment(0,3) = (t1.translation()+t2.translation())/2;

        Eigen::VectorXd val;
        val.resize(1);
    //    double distance_rotation1 = distanceRotations(X_center.segment(3,4), X_center_d.segment(3,4));
        val(0) = (X_center.segment(0,3)-X_center_d.segment(0,3)).transpose()*(X_center.segment(0,3)-X_center_d.segment(0,3));
    //    val = (X_center.segment(0,3)-X_center_d.segment(0,3));
    //    val(0) = std::sqrt(val(0));
    //    val(0) += distance_rotation1*distance_rotation1;
    //    std::cout << "constraint val:" << val.transpose() << std::endl;
    //    std::cout << "X_center:" << X_center.transpose() << std::endl;
        return val;
}

Eigen::MatrixXd DualArmCenterDesiredPoseConstraint::calculateConstraintDerivativeAA(Eigen::VectorXd &vec)
{

}

Eigen::VectorXd DualArmCenterDesiredPoseConstraint::getLowerBounds()
{
    Eigen::VectorXd lb;
    lb.resize(1);
    lb << 0;
    return lb;
}

Eigen::VectorXd DualArmCenterDesiredPoseConstraint::getUpperBounds()
{
    Eigen::VectorXd ub;
    ub.resize(1);
    ub << 1e-6;
    return ub;
}

void DualArmCenterDesiredPoseConstraint::updateConstraintParams()
{

}


CollisionConstraintPosition::CollisionConstraintPosition():
    Constraint(Configuration, Position, "collision_constraint"),
    distance_threshold(0.05)
{

}

CollisionConstraintPosition::CollisionConstraintPosition(rl::mdl::Kinematic *kin, std::string fname):
    Constraint(Configuration, Position, "collision_constraint"),
    collisionScene(new rl::sg::solid::Scene()),
    kinematic(kin),
    distance_threshold(0.05)
{
    collisionScene->load(fname, true, true);
}

CollisionConstraintPosition::CollisionConstraintPosition(rl::mdl::Kinematic *kin):
    Constraint(Configuration, Position, "collision_constraint"),
    kinematic(kin),
    distance_threshold(0.05)
{

}

int CollisionConstraintPosition::getNumConstraints()
{
    return collision_bodies.size()*kinematic->getBodies();
}

Eigen::VectorXd CollisionConstraintPosition::calculateConstraintValue(Eigen::VectorXd &vec)
{
    Eigen::VectorXd val;
    val.resize(collision_bodies.size()*kinematic->getBodies());
    kinematic->setPosition(vec);
    kinematic->forwardPosition();
    for(std::size_t collision_body_id = 0;collision_body_id < collision_bodies.size();++collision_body_id)
    {
        Eigen::MatrixXd points_body = points[collision_body_id];
        for(std::size_t body_id = 0;body_id < kinematic->getBodies();body_id++)
        {
            Eigen::Vector3d point_robot_body = kinematic->getBody(body_id)->t.translation();
            Eigen::Vector3d point_robot_body_closest = point_robot_body+points_body.block(body_id,0,1,3).transpose();
            Eigen::Vector3d point_collision_body_closest = points_body.block(body_id,3,1,3).transpose();
            double distance_min = (point_collision_body_closest-point_robot_body_closest).squaredNorm();
            val(collision_body_id*kinematic->getBodies()+body_id) = distance_min;
//            std::cout << distance_min << " ";
//            if(distance_min < 0.05*0.05)
//            {
//                val(collision_body_id*kinematic->getBodies()+body_id) = distance_min-0.02*0.02;
//            }
//            else
//                val(collision_body_id*kinematic->getBodies()+body_id) = 0;
        }
    }
//    std::cout << "\n";
    return val;
}

Eigen::MatrixXd CollisionConstraintPosition::calculateConstraintDerivative(Eigen::VectorXd &vec)
{
    Eigen::MatrixXd val;
    val.resize(getNumConstraints(),vec.size());
    Eigen::VectorXd dX_minus, dX_plus;
    dX_minus.resize(vec.size());
    dX_plus.resize(vec.size());
    double eps = 1e-6;
    for(std::size_t dx_id = 0;dx_id < vec.size();++dx_id)
    {
        dX_minus = vec;
        dX_plus = vec;
        dX_minus(dx_id) -= eps;
        dX_plus(dx_id) += eps;
        Eigen::VectorXd val_minus = calculateConstraintValue(dX_minus);
        Eigen::VectorXd val_plus = calculateConstraintValue(dX_plus);
        val.col(dx_id) = (val_plus-val_minus)/(2.0*eps);
    }
    return val;
}

Eigen::VectorXd CollisionConstraintPosition::getLowerBounds()
{
    Eigen::VectorXd val;
    val.resize(kinematic->getBodies()*collision_bodies.size());
    for(std::size_t constraint_id = 0;constraint_id < kinematic->getBodies()*collision_bodies.size();++constraint_id)
    {
        val(constraint_id) = distance_threshold*distance_threshold;
    }
    return val;
}

Eigen::VectorXd CollisionConstraintPosition::getUpperBounds()
{
    Eigen::VectorXd val;
    val.resize(kinematic->getBodies()*collision_bodies.size());
    for(std::size_t constraint_id = 0;constraint_id < kinematic->getBodies()*collision_bodies.size();++constraint_id)
    {
        val(constraint_id) = 1e19;
    }
    return val;
}

void CollisionConstraintPosition::updateConstraintParams()
{
    points.clear();
    for(std::size_t collision_body_id = 0;collision_body_id < collision_bodies.size();++collision_body_id)
    {
        points.push_back(getCollision(collision_bodies[collision_body_id]));
    }
    q_curr.resize(kinematic->getDof());
    kinematic->getPosition(q_curr);
}

Eigen::MatrixXd CollisionConstraintPosition::getCollision(rl::sg::Body *body)
{
//    const clock_t begin_time = clock();
    Eigen::MatrixXd points;
    points.resize(kinematic->getBodies(), 6);
    if (rl::sg::DistanceScene* distanceScene = dynamic_cast< rl::sg::DistanceScene* >(collisionScene.get()))
    {
        rl::math::Vector3 point1;
        rl::math::Vector3 point2;
        for(std::size_t body_id = 0;body_id < kinematic->getBodies();++body_id)
        {
            collisionScene->getModel(0)->getBody(body_id)->setFrame(kinematic->getBody(body_id)->t);
            rl::math::Real distance = distanceScene->distance(
                        collisionScene->getModel(0)->getBody(body_id),
                        body,
                        point1,
                        point2
                        );
//                        std::cout << point1 << std::endl;
//                        std::cout << point2 << std::endl;
//                        std::cout << distance << std::endl;
//                        std::cout << (point1-point2).transpose() << std::endl;
            points.block(body_id, 0, 1, 3) = (point1-kinematic->getBody(body_id)->t.translation()).transpose();
            points.block(body_id, 3, 1, 3) = point2.transpose();
        }
    }
//    std::cout << "collision time: " << double( clock () - begin_time ) /  CLOCKS_PER_SEC << std::endl;

    return points;
}


CollisionConstraintExact::CollisionConstraintExact():
    CollisionConstraintPosition()
{
}

CollisionConstraintExact::CollisionConstraintExact(rl::mdl::Kinematic *kin, std::string fname):
    CollisionConstraintPosition(kin, fname)
{
}

CollisionConstraintExact::CollisionConstraintExact(rl::mdl::Kinematic *kin):
    CollisionConstraintPosition(kin)
{
}

Eigen::VectorXd CollisionConstraintExact::calculateConstraintValue(Eigen::VectorXd &vec)
{
    Eigen::VectorXd val;
    val.resize(collision_bodies.size()*kinematic->getBodies());
    kinematic->setPosition(vec);
    kinematic->forwardPosition();
    points.clear();
    for(std::size_t collision_body_id = 0;collision_body_id < collision_bodies.size();++collision_body_id)
    {
        points.push_back(getCollision(collision_bodies[collision_body_id]));
    }
    for(std::size_t collision_body_id = 0;collision_body_id < collision_bodies.size();++collision_body_id)
    {
        Eigen::MatrixXd points_body = points[collision_body_id];
        for(std::size_t body_id = 0;body_id < kinematic->getBodies();++body_id)
        {
            Eigen::Vector3d point_robot_body = kinematic->getBody(body_id)->t.translation();
            Eigen::Vector3d point_robot_body_closest = point_robot_body+points_body.block(body_id,0,1,3).transpose();
            Eigen::Vector3d point_collision_body_closest = points_body.block(body_id,3,1,3).transpose();
            double distance_min = (point_collision_body_closest-point_robot_body_closest).squaredNorm();
            val(collision_body_id*kinematic->getBodies()+body_id) = distance_min;
        }
    }
    return val;
}
