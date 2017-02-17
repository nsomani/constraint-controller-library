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


#include "GeometricConstraints.h"

PlanePlaneDistanceMinMaxConstraint::PlanePlaneDistanceMinMaxConstraint():
    OperationalPositionConstraint(std::string("plane_plane_distance_min_max"))
{

}

PlanePlaneDistanceMinMaxConstraint::PlanePlaneDistanceMinMaxConstraint(PlaneShapeDescriptor *fixed, PlaneShapeDescriptor *constrained, double d_min, double d_max, bool maximize_d, bool minimize_d):
    OperationalPositionConstraint(std::string("plane_plane_distance_min_max")),
    plane_fixed(fixed),
    plane_constrained(constrained),
    distance_max(d_max),
    distance_min(d_min),
    maximize_distance(maximize_d),
    minimize_distance(minimize_d)
{
}

int PlanePlaneDistanceMinMaxConstraint::getNumConstraints()
{
    return 2;
}

Eigen::VectorXd PlanePlaneDistanceMinMaxConstraint::calculateConstraintValue(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed;
    Eigen::Matrix3d R_fixed;
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        R_fixed = Eigen::Quaterniond(vec(13), vec(10), vec(11), vec(12)).toRotationMatrix();
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    t = vec.segment(0,3);
    R = Eigen::Quaterniond(vec(6), vec(3), vec(4), vec(5)).toRotationMatrix();
    Eigen::VectorXd val;
    val.resize(2);
    val(0) = (R*plane_constrained->point->point+t-(R_fixed*plane_fixed->point->point+t_fixed)).dot(R_fixed*plane_fixed->normal_direction->vec);
    val(1) = (R*plane_constrained->normal_direction->vec).dot((R_fixed*plane_fixed->normal_direction->vec));
    return val;
}

Eigen::VectorXd PlanePlaneDistanceMinMaxConstraint::calculateConstraintValueAA(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed;
    Eigen::Matrix3d R_fixed;
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        Eigen::Vector3d w_fixed = vec.segment(10,3);
        R_fixed = Eigen::AngleAxisd(vec(13), w_fixed);
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    t = vec.segment(0,3);
    Eigen::Vector3d w = vec.segment(3,3);
    R = Eigen::AngleAxisd(vec(6), w);
    Eigen::VectorXd val;
    val.resize(2);
    val(0) = (R*plane_constrained->point->point+t-(R_fixed*plane_fixed->point->point+t_fixed)).dot(R_fixed*plane_fixed->normal_direction->vec);
    val(1) = (R*plane_constrained->normal_direction->vec).dot((R_fixed*plane_fixed->normal_direction->vec));
    return val;
}

Eigen::MatrixXd PlanePlaneDistanceMinMaxConstraint::calculateConstraintDerivativeAA(Eigen::VectorXd &vec)
{

}

Eigen::MatrixXd PlanePlaneDistanceMinMaxConstraint::calculateConstraintDerivative(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed, w_fixed;
    Eigen::Matrix3d R_fixed;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        w_fixed = vec.segment(10,3);
        R_fixed = Eigen::AngleAxisd(vec(13), w_fixed);
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    Eigen::Vector4d axisAngle = vec.segment(3,4);
    Eigen::MatrixXd val;
    val.resize(2,7);
    val(0,0)=(R_fixed*plane_fixed->normal_direction->vec)(0);
    val(0,1)=(R_fixed*plane_fixed->normal_direction->vec)(1);
    val(0,2)=(R_fixed*plane_fixed->normal_direction->vec)(2);
    val(0,3)=(R_fixed*plane_fixed->normal_direction->vec).transpose()*getdRx(axisAngle)*plane_constrained->point->point;
    val(0,4)=(R_fixed*plane_fixed->normal_direction->vec).transpose()*getdRy(axisAngle)*plane_constrained->point->point;
    val(0,5)=(R_fixed*plane_fixed->normal_direction->vec).transpose()*getdRz(axisAngle)*plane_constrained->point->point;
    val(0,6)=(R_fixed*plane_fixed->normal_direction->vec).transpose()*getdRtheta(axisAngle)*plane_constrained->point->point;
    val(1,0)=0;
    val(1,1)=0;
    val(1,2)=0;
    val(1,3)=(R_fixed*plane_fixed->normal_direction->vec).transpose()*getdRx(axisAngle)*plane_constrained->normal_direction->vec;
    val(1,4)=(R_fixed*plane_fixed->normal_direction->vec).transpose()*getdRy(axisAngle)*plane_constrained->normal_direction->vec;
    val(1,5)=(R_fixed*plane_fixed->normal_direction->vec).transpose()*getdRz(axisAngle)*plane_constrained->normal_direction->vec;
    val(1,6)=(R_fixed*plane_fixed->normal_direction->vec).transpose()*getdRtheta(axisAngle)*plane_constrained->normal_direction->vec;
    return val;
}

Eigen::VectorXd PlanePlaneDistanceMinMaxConstraint::getLowerBounds()
{
    Eigen::VectorXd lb;
    lb.resize(2);
    lb << distance_min,1.0;
    return lb;
}

Eigen::VectorXd PlanePlaneDistanceMinMaxConstraint::getUpperBounds()
{
    Eigen::VectorXd ub;
    ub.resize(2);
    ub << distance_max,1.0;
    return ub;
}

PlanePlaneDistanceAngleMinMaxConstraint::PlanePlaneDistanceAngleMinMaxConstraint():
    OperationalPositionConstraint(std::string("plane_plane_distance_angle_min_max"))
{

}

PlanePlaneDistanceAngleMinMaxConstraint::PlanePlaneDistanceAngleMinMaxConstraint(PlaneShapeDescriptor *fixed, PlaneShapeDescriptor *constrained, double d_min, double d_max, double a_min, double a_max, bool maximize_d, bool minimize_d):
    OperationalPositionConstraint(std::string("plane_plane_distance_angle_min_max")),
    plane_fixed(fixed),
    plane_constrained(constrained),
    distance_max(d_max),
    distance_min(d_min),
    angle_min(a_min),
    angle_max(a_max),
    maximize_distance(maximize_d),
    minimize_distance(minimize_d)
{

}

int PlanePlaneDistanceAngleMinMaxConstraint::getNumConstraints()
{
    return 2;
}

Eigen::VectorXd PlanePlaneDistanceAngleMinMaxConstraint::calculateConstraintValue(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed;
    Eigen::Matrix3d R_fixed;
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        R_fixed = Eigen::Quaterniond(vec(13), vec(10), vec(11), vec(12)).toRotationMatrix();
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    t = vec.segment(0,3);
    R = Eigen::Quaterniond(vec(6), vec(3), vec(4), vec(5)).toRotationMatrix();
    Eigen::VectorXd val;
    val.resize(2);
    val(0) = (R*plane_constrained->point->point+t-(R_fixed*plane_fixed->point->point+t_fixed)).dot(R_fixed*plane_fixed->normal_direction->vec);
    val(1) = (R*plane_constrained->normal_direction->vec).dot(R_fixed*plane_fixed->normal_direction->vec);
    return val;
}

Eigen::VectorXd PlanePlaneDistanceAngleMinMaxConstraint::calculateConstraintValueAA(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed;
    Eigen::Matrix3d R_fixed;
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        Eigen::Vector3d w_fixed = vec.segment(10,3);
        R_fixed = Eigen::AngleAxisd(vec(13), w_fixed);
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    t = vec.segment(0,3);
    Eigen::Vector3d w = vec.segment(3,3);
    R = Eigen::AngleAxisd(vec(6), w);
    Eigen::VectorXd val;
    val.resize(2);
    val(0) = (R*plane_constrained->point->point+t-(R_fixed*plane_fixed->point->point+t_fixed)).dot(R_fixed*plane_fixed->normal_direction->vec);
    val(1) = (R*plane_constrained->normal_direction->vec).dot(R_fixed*plane_fixed->normal_direction->vec);
    return val;
}

Eigen::MatrixXd PlanePlaneDistanceAngleMinMaxConstraint::calculateConstraintDerivativeAA(Eigen::VectorXd &vec)
{

}

Eigen::MatrixXd PlanePlaneDistanceAngleMinMaxConstraint::calculateConstraintDerivative(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed, w_fixed;
    Eigen::Matrix3d R_fixed;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        w_fixed = vec.segment(10,3);
        R_fixed = Eigen::AngleAxisd(vec(13), w_fixed);
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    Eigen::Vector4d axisAngle = vec.segment(3,4);
    Eigen::MatrixXd val;
    val.resize(2,7);
    val(0,0)=(R_fixed*plane_fixed->normal_direction->vec)(0);
    val(0,1)=(R_fixed*plane_fixed->normal_direction->vec)(1);
    val(0,2)=(R_fixed*plane_fixed->normal_direction->vec)(2);
    val(0,3)=(R_fixed*plane_fixed->normal_direction->vec).transpose()*getdRx(axisAngle)*plane_constrained->point->point;
    val(0,4)=(R_fixed*plane_fixed->normal_direction->vec).transpose()*getdRy(axisAngle)*plane_constrained->point->point;
    val(0,5)=(R_fixed*plane_fixed->normal_direction->vec).transpose()*getdRz(axisAngle)*plane_constrained->point->point;
    val(0,6)=(R_fixed*plane_fixed->normal_direction->vec).transpose()*getdRtheta(axisAngle)*plane_constrained->point->point;
    val(1,0)=0;
    val(1,1)=0;
    val(1,2)=0;
    val(1,3)=(R_fixed*plane_fixed->normal_direction->vec).transpose()*getdRx(axisAngle)*plane_constrained->normal_direction->vec;
    val(1,4)=(R_fixed*plane_fixed->normal_direction->vec).transpose()*getdRy(axisAngle)*plane_constrained->normal_direction->vec;
    val(1,5)=(R_fixed*plane_fixed->normal_direction->vec).transpose()*getdRz(axisAngle)*plane_constrained->normal_direction->vec;
    val(1,6)=(R_fixed*plane_fixed->normal_direction->vec).transpose()*getdRtheta(axisAngle)*plane_constrained->normal_direction->vec;
    return val;
}

Eigen::VectorXd PlanePlaneDistanceAngleMinMaxConstraint::getLowerBounds()
{
    Eigen::VectorXd lb;
    lb.resize(2);
    lb << distance_min,std::cos(angle_max);
    return lb;
}

Eigen::VectorXd PlanePlaneDistanceAngleMinMaxConstraint::getUpperBounds()
{
    Eigen::VectorXd ub;
    ub.resize(2);
    ub << distance_max,std::cos(angle_min);
    return ub;
}

PointPlaneDistanceMinMaxConstraint::PointPlaneDistanceMinMaxConstraint():
    OperationalPositionConstraint(std::string("point_plane_distance_minx_max"))
{

}

PointPlaneDistanceMinMaxConstraint::PointPlaneDistanceMinMaxConstraint(PointShapeDescriptor *fixed, PlaneShapeDescriptor *constrained, double d_min, double d_max, bool maximize_d, bool minimize_d):
    OperationalPositionConstraint(std::string("point_plane_distance_minx_max")),
    point_fixed(fixed),
    plane_constrained(constrained),
    distance_max(d_max),
    distance_min(d_min),
    maximize_distance(maximize_d),
    minimize_distance(minimize_d)
{

}

int PointPlaneDistanceMinMaxConstraint::getNumConstraints()
{
    return 1;
}

Eigen::VectorXd PointPlaneDistanceMinMaxConstraint::calculateConstraintValue(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed;
    Eigen::Matrix3d R_fixed;
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        R_fixed = Eigen::Quaterniond(vec(13), vec(10), vec(11), vec(12)).toRotationMatrix();
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    t = vec.segment(0,3);
    R = Eigen::Quaterniond(vec(6), vec(3), vec(4), vec(5)).toRotationMatrix();
    Eigen::VectorXd val;
    val.resize(1);
    val(0) = (R*plane_constrained->normal_direction->vec).transpose()*(R*plane_constrained->point->point+t-(R_fixed*point_fixed->point+t_fixed));

    return val;
}

Eigen::VectorXd PointPlaneDistanceMinMaxConstraint::calculateConstraintValueAA(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed;
    Eigen::Matrix3d R_fixed;
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        Eigen::Vector3d w_fixed = vec.segment(10,3);
        R_fixed = Eigen::AngleAxisd(vec(13), w_fixed);
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    t = vec.segment(0,3);
    Eigen::Vector3d w = vec.segment(3,3);
    R = Eigen::AngleAxisd(vec(6), w);
    Eigen::VectorXd val;
    val.resize(1);
    val(0) = (R*plane_constrained->normal_direction->vec).transpose()*(R*plane_constrained->point->point+t-(R_fixed*point_fixed->point+t_fixed));

    return val;
}

Eigen::MatrixXd PointPlaneDistanceMinMaxConstraint::calculateConstraintDerivativeAA(Eigen::VectorXd &vec)
{

}

Eigen::MatrixXd PointPlaneDistanceMinMaxConstraint::calculateConstraintDerivative(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed, w_fixed;
    Eigen::Matrix3d R_fixed;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        w_fixed = vec.segment(10,3);
        R_fixed = Eigen::AngleAxisd(vec(13), w_fixed);
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    Eigen::Vector3d t = vec.segment(0,3);
    Eigen::Vector3d w = vec.segment(3,3);
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(vec(6), w);
    Eigen::Vector4d axisAngle = vec.segment(3,4);
    Eigen::MatrixXd val;
    val.resize(1,7);
    val(0,0)=(R*plane_constrained->normal_direction->vec)(0);
    val(0,1)=(R*plane_constrained->normal_direction->vec)(1);
    val(0,2)=(R*plane_constrained->normal_direction->vec)(2);
    val(0,3)=(t-(R_fixed*point_fixed->point+t_fixed)).transpose()*getdRx(axisAngle)*plane_constrained->normal_direction->vec;
    val(0,4)=(t-(R_fixed*point_fixed->point+t_fixed)).transpose()*getdRy(axisAngle)*plane_constrained->normal_direction->vec;
    val(0,5)=(t-(R_fixed*point_fixed->point+t_fixed)).transpose()*getdRz(axisAngle)*plane_constrained->normal_direction->vec;
    val(0,6)=(t-(R_fixed*point_fixed->point+t_fixed)).transpose()*getdRtheta(axisAngle)*plane_constrained->normal_direction->vec;

    return val;
}

Eigen::VectorXd PointPlaneDistanceMinMaxConstraint::getLowerBounds()
{
    Eigen::VectorXd lb;
    lb.resize(1);
    lb << distance_min;
    return lb;
}

Eigen::VectorXd PointPlaneDistanceMinMaxConstraint::getUpperBounds()
{
    Eigen::VectorXd ub;
    ub.resize(1);
    ub << distance_max;
    return ub;
}

PointPlaneCoincidentConstraint::PointPlaneCoincidentConstraint(PointShapeDescriptor *fixed, PlaneShapeDescriptor *constrained):
    PointPlaneDistanceMinMaxConstraint(fixed, constrained, 0, 0)
{
}

CylinderCylinderParallelDistanceMinMaxConstraint::CylinderCylinderParallelDistanceMinMaxConstraint():
    OperationalPositionConstraint(std::string("cylinder_cylinder_parallel_distance_minx_max"))
{

}

CylinderCylinderParallelDistanceMinMaxConstraint::CylinderCylinderParallelDistanceMinMaxConstraint(CylinderShapeDescriptor *fixed, CylinderShapeDescriptor *constrained, double d_min, double d_max):
    OperationalPositionConstraint(std::string("cylinder_cylinder_parallel_distance_minx_max")),
    cylinder_fixed(fixed),
    cylinder_constrained(constrained),
    distance_max(d_max),
    distance_min(d_min)
{
}

int CylinderCylinderParallelDistanceMinMaxConstraint::getNumConstraints()
{
    return 2;
}

Eigen::VectorXd CylinderCylinderParallelDistanceMinMaxConstraint::calculateConstraintValue(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed;
    Eigen::Matrix3d R_fixed;
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        R_fixed = Eigen::Quaterniond(vec(13), vec(10), vec(11), vec(12)).toRotationMatrix();
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    t = vec.segment(0,3);
    R = Eigen::Quaterniond(vec(6), vec(3), vec(4), vec(5)).toRotationMatrix();
    Eigen::VectorXd val;
    val.resize(2);
    Eigen::Vector3d p2_t = R*cylinder_constrained->center->point+t;
    Eigen::Vector3d cylinder_min_point = (R_fixed*cylinder_fixed->center->point+t_fixed) + ((R_fixed*cylinder_fixed->axis->vec).dot(p2_t-(R_fixed*cylinder_fixed->center->point+t_fixed)))*(R_fixed*cylinder_fixed->axis->vec);
    Eigen::Vector3d d = p2_t - cylinder_min_point;
    val(0) = d.transpose()*d;
    val(1) = (R*cylinder_constrained->axis->vec).dot(R_fixed*cylinder_fixed->axis->vec);

    return val;
}

Eigen::VectorXd CylinderCylinderParallelDistanceMinMaxConstraint::calculateConstraintValueAA(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed;
    Eigen::Matrix3d R_fixed;
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        Eigen::Vector3d w_fixed = vec.segment(10,3);
        R_fixed = Eigen::AngleAxisd(vec(13), w_fixed);
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    t = vec.segment(0,3);
    Eigen::Vector3d w = vec.segment(3,3);
    R = Eigen::AngleAxisd(vec(6), w);
    Eigen::VectorXd val;
    val.resize(2);
    Eigen::Vector3d p2_t = R*cylinder_constrained->center->point+t;
    Eigen::Vector3d cylinder_min_point = (R_fixed*cylinder_fixed->center->point+t_fixed) + ((R_fixed*cylinder_fixed->axis->vec).dot(p2_t-(R_fixed*cylinder_fixed->center->point+t_fixed)))*(R_fixed*cylinder_fixed->axis->vec);
    Eigen::Vector3d d = p2_t - cylinder_min_point;
    val(0) = d.transpose()*d;
    val(1) = (R*cylinder_constrained->axis->vec).dot(R_fixed*cylinder_fixed->axis->vec);

    return val;
}

Eigen::MatrixXd CylinderCylinderParallelDistanceMinMaxConstraint::calculateConstraintDerivativeAA(Eigen::VectorXd &vec)
{

}

Eigen::MatrixXd CylinderCylinderParallelDistanceMinMaxConstraint::calculateConstraintDerivative(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed, w_fixed;
    Eigen::Matrix3d R_fixed;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        w_fixed = vec.segment(10,3);
        R_fixed = Eigen::AngleAxisd(vec(13), w_fixed);
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    Eigen::Vector3d t = vec.segment(0,3);
    Eigen::Vector3d w = vec.segment(3,3);
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(vec(6), w);
    Eigen::Vector3d p2_t = R*cylinder_constrained->center->point+t;
    Eigen::Vector3d cylinder_min_point = (R_fixed*cylinder_fixed->center->point+t_fixed) + ((R_fixed*cylinder_fixed->axis->vec).dot(p2_t-(R_fixed*cylinder_fixed->center->point+t_fixed)))*(R_fixed*cylinder_fixed->axis->vec);
    Eigen::Vector3d d = p2_t - cylinder_min_point;

    Eigen::VectorXd val;
    val.resize(2,7);
    val(0,0) = 2*d(0);
    val(0,1) = 2*d(1);
    val(0,2) = 2*d(2);
    val(0,3) = 2*d.transpose()*getdRx(vec.segment(3,4))*cylinder_constrained->center->point;
    val(0,4) = 2*d.transpose()*getdRy(vec.segment(3,4))*cylinder_constrained->center->point;
    val(0,5) = 2*d.transpose()*getdRz(vec.segment(3,4))*cylinder_constrained->center->point;
    val(0,6) = 2*d.transpose()*getdRtheta(vec.segment(3,4))*cylinder_constrained->center->point;
    val(1,0) = 0;
    val(1,1) = 0;
    val(1,2) = 0;
    val(1,3) = (R_fixed*cylinder_fixed->axis->vec).transpose()*getdRx(vec.segment(3,4))*cylinder_constrained->axis->vec;
    val(1,4) = (R_fixed*cylinder_fixed->axis->vec).transpose()*getdRy(vec.segment(3,4))*cylinder_constrained->axis->vec;
    val(1,5) = (R_fixed*cylinder_fixed->axis->vec).transpose()*getdRz(vec.segment(3,4))*cylinder_constrained->axis->vec;
    val(1,6) = (R_fixed*cylinder_fixed->axis->vec).transpose()*getdRtheta(vec.segment(3,4))*cylinder_constrained->axis->vec;

    return val;

}

Eigen::VectorXd CylinderCylinderParallelDistanceMinMaxConstraint::getLowerBounds()
{
    Eigen::VectorXd lb;
    lb.resize(2);
    lb << distance_min*distance_min,1.0;
    return lb;
}

Eigen::VectorXd CylinderCylinderParallelDistanceMinMaxConstraint::getUpperBounds()
{
    Eigen::VectorXd ub;
    ub.resize(2);
    ub << distance_max*distance_max,1.0;
    return ub;
}

CylinderCylinderParallelDistanceConstraint::CylinderCylinderParallelDistanceConstraint(CylinderShapeDescriptor *fixed, CylinderShapeDescriptor *constrained, double dist):
    CylinderCylinderParallelDistanceMinMaxConstraint(fixed, constrained, dist, dist)
{
    this->OperationalPositionConstraint::constraint_name = "cylinder_cylinder_parallel_distance";
}

CylinderCylinderParallelInsideConstraint::CylinderCylinderParallelInsideConstraint(CylinderShapeDescriptor *fixed, CylinderShapeDescriptor *constrained):
    CylinderCylinderParallelDistanceMinMaxConstraint(fixed, constrained, 0, std::fabs(fixed->radius - constrained->radius))
{

}

PlanePlaneDistanceConstraint::PlanePlaneDistanceConstraint(PlaneShapeDescriptor *fixed, PlaneShapeDescriptor *constrained, double d, bool maximize_d, bool minimize_d):
    PlanePlaneDistanceMinMaxConstraint(fixed, constrained, d, d, maximize_d, minimize_d), distance(d)
{
    this->OperationalPositionConstraint::constraint_name = "plane_plane_distance";
}

PlanePlaneSideConstraint::PlanePlaneSideConstraint(PlaneShapeDescriptor *fixed, PlaneShapeDescriptor *constrained):
    PlanePlaneDistanceMinMaxConstraint(fixed, constrained, 0, 1e19)
{

}

PlanePointDistanceMinMaxConstraint::PlanePointDistanceMinMaxConstraint():
    OperationalPositionConstraint(std::string("plane_point_distance_minx_max"))
{

}

PlanePointDistanceMinMaxConstraint::PlanePointDistanceMinMaxConstraint(PlaneShapeDescriptor *fixed, PointShapeDescriptor *constrained, double d_min, double d_max, bool maximize_d, bool minimize_d):
    OperationalPositionConstraint(std::string("point_plane_distance_minx_max")),
    plane_fixed(fixed),
    point_constrained(constrained),
    distance_max(d_max),
    distance_min(d_min),
    maximize_distance(maximize_d),
    minimize_distance(minimize_d)
{

}

int PlanePointDistanceMinMaxConstraint::getNumConstraints()
{
    return 1;
}

Eigen::VectorXd PlanePointDistanceMinMaxConstraint::calculateConstraintValue(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed;
    Eigen::Matrix3d R_fixed;
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        R_fixed = Eigen::Quaterniond(vec(13), vec(10), vec(11), vec(12)).toRotationMatrix();
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    t = vec.segment(0,3);
    R = Eigen::Quaterniond(vec(6), vec(3), vec(4), vec(5)).toRotationMatrix();
    Eigen::VectorXd val;
    val.resize(1);
    val(0) = (R_fixed*plane_fixed->normal_direction->vec).transpose()*((R_fixed*plane_fixed->point->point+t_fixed)-t-R*point_constrained->point);

    return val;
}

Eigen::VectorXd PlanePointDistanceMinMaxConstraint::calculateConstraintValueAA(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed;
    Eigen::Matrix3d R_fixed;
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        Eigen::Vector3d w_fixed = vec.segment(10,3);
        R_fixed = Eigen::AngleAxisd(vec(13), w_fixed);
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    t = vec.segment(0,3);
    Eigen::Vector3d w = vec.segment(3,3);
    R = Eigen::AngleAxisd(vec(6), w);
    Eigen::VectorXd val;
    val.resize(1);
    val(0) = (R_fixed*plane_fixed->normal_direction->vec).transpose()*((R_fixed*plane_fixed->point->point+t_fixed)-t-R*point_constrained->point);

    return val;
}

Eigen::MatrixXd PlanePointDistanceMinMaxConstraint::calculateConstraintDerivativeAA(Eigen::VectorXd &vec)
{

}

Eigen::MatrixXd PlanePointDistanceMinMaxConstraint::calculateConstraintDerivative(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed, w_fixed;
    Eigen::Matrix3d R_fixed;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        w_fixed = vec.segment(10,3);
        R_fixed = Eigen::AngleAxisd(vec(13), w_fixed);
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    Eigen::Vector3d w = vec.segment(3,3);
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(vec(6), w);
    Eigen::MatrixXd val;
    val.resize(1,7);
    val(0,0)=(R_fixed*plane_fixed->normal_direction->vec)(0);
    val(0,1)=(R_fixed*plane_fixed->normal_direction->vec)(1);
    val(0,2)=(R_fixed*plane_fixed->normal_direction->vec)(2);
    val(0,3)=(R_fixed*plane_fixed->normal_direction->vec).transpose()*getdRx(vec.segment(3,4))*point_constrained->point;
    val(0,4)=(R_fixed*plane_fixed->normal_direction->vec).transpose()*getdRy(vec.segment(3,4))*point_constrained->point;
    val(0,5)=(R_fixed*plane_fixed->normal_direction->vec).transpose()*getdRz(vec.segment(3,4))*point_constrained->point;
    val(0,6)=(R_fixed*plane_fixed->normal_direction->vec).transpose()*getdRtheta(vec.segment(3,4))*point_constrained->point;
}

Eigen::VectorXd PlanePointDistanceMinMaxConstraint::getLowerBounds()
{
    Eigen::VectorXd lb;
    lb.resize(1);
    lb << distance_min;
    return lb;
}

Eigen::VectorXd PlanePointDistanceMinMaxConstraint::getUpperBounds()
{
    Eigen::VectorXd ub;
    ub.resize(1);
    ub << distance_max;
    return ub;
}

PlanePointDistanceConstraint::PlanePointDistanceConstraint():
    PlanePointDistanceMinMaxConstraint()
{

}

PlanePointDistanceConstraint::PlanePointDistanceConstraint(PlaneShapeDescriptor *fixed, PointShapeDescriptor *constrained, double d):
    PlanePointDistanceMinMaxConstraint(fixed, constrained, distance, distance),
    distance(d)
{
    plane_fixed = fixed;
    point_constrained = constrained;
}

PlanePlaneAngleMinMaxConstraint::PlanePlaneAngleMinMaxConstraint():
    OperationalPositionConstraint(std::string("plane_plane_angle"))
{

}

PlanePlaneAngleMinMaxConstraint::PlanePlaneAngleMinMaxConstraint(PlaneShapeDescriptor *fixed, PlaneShapeDescriptor *constrained, double a_min, double a_max, bool maximize_a, bool minimize_a):
    OperationalPositionConstraint(std::string("plane_plane_angle")),
    plane_fixed(fixed),
    plane_constrained(constrained),
    angle_min(a_min),
    angle_max(a_max),
    maximize_angle(maximize_a),
    minimize_angle(minimize_a)
{
    this->OperationalPositionConstraint::constraint_name = "plane_plane_angle_min_max";
}

int PlanePlaneAngleMinMaxConstraint::getNumConstraints()
{
    return 1;
}

Eigen::VectorXd PlanePlaneAngleMinMaxConstraint::calculateConstraintValue(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed;
    Eigen::Matrix3d R_fixed;
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        R_fixed = Eigen::Quaterniond(vec(13), vec(10), vec(11), vec(12)).toRotationMatrix();
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    t = vec.segment(0,3);
    R = Eigen::Quaterniond(vec(6), vec(3), vec(4), vec(5)).toRotationMatrix();
    Eigen::VectorXd val;
    val.resize(1);
    val(0) = (R*plane_constrained->normal_direction->vec).dot(R_fixed*plane_fixed->normal_direction->vec);
    return val;
}

Eigen::VectorXd PlanePlaneAngleMinMaxConstraint::calculateConstraintValueAA(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed;
    Eigen::Matrix3d R_fixed;
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        Eigen::Vector3d w_fixed = vec.segment(10,3);
        R_fixed = Eigen::AngleAxisd(vec(13), w_fixed);
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    t = vec.segment(0,3);
    Eigen::Vector3d w = vec.segment(3,3);
    R = Eigen::AngleAxisd(vec(6), w);
    Eigen::VectorXd val;
    val.resize(1);
    val(0) = (R*plane_constrained->normal_direction->vec).dot(R_fixed*plane_fixed->normal_direction->vec);
    return val;
}

Eigen::MatrixXd PlanePlaneAngleMinMaxConstraint::calculateConstraintDerivativeAA(Eigen::VectorXd &vec)
{

}

Eigen::MatrixXd PlanePlaneAngleMinMaxConstraint::calculateConstraintDerivative(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed, w_fixed;
    Eigen::Matrix3d R_fixed;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        w_fixed = vec.segment(10,3);
        R_fixed = Eigen::AngleAxisd(vec(13), w_fixed);
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    Eigen::Vector4d axisAngle = vec.segment(3,4);
    Eigen::MatrixXd val;
    val.resize(1,7);
    val(0,0)=0;
    val(0,1)=0;
    val(0,2)=0;
    val(0,3)=(R_fixed*plane_fixed->normal_direction->vec).transpose()*getdRx(axisAngle)*plane_constrained->normal_direction->vec;
    val(0,4)=(R_fixed*plane_fixed->normal_direction->vec).transpose()*getdRy(axisAngle)*plane_constrained->normal_direction->vec;
    val(0,5)=(R_fixed*plane_fixed->normal_direction->vec).transpose()*getdRz(axisAngle)*plane_constrained->normal_direction->vec;
    val(0,6)=(R_fixed*plane_fixed->normal_direction->vec).transpose()*getdRtheta(axisAngle)*plane_constrained->normal_direction->vec;
    return val;
}

Eigen::VectorXd PlanePlaneAngleMinMaxConstraint::getLowerBounds()
{
    Eigen::VectorXd lb;
    lb.resize(1);
    lb << std::cos(angle_min);
    return lb;
}

Eigen::VectorXd PlanePlaneAngleMinMaxConstraint::getUpperBounds()
{
    Eigen::VectorXd ub;
    ub.resize(1);
    ub << std::cos(angle_max);
    return ub;
}

PlanePlaneParallelConstraint::PlanePlaneParallelConstraint():
    PlanePlaneAngleMinMaxConstraint()
{

}

PlanePlaneParallelConstraint::PlanePlaneParallelConstraint(PlaneShapeDescriptor *fixed, PlaneShapeDescriptor *constrained):
    PlanePlaneAngleMinMaxConstraint(fixed, constrained, 0, 0)
{
    this->OperationalPositionConstraint::constraint_name = "plane_plane_parallel";
}

LinePointCoincidentConstraint::LinePointCoincidentConstraint(LineShapeDescriptor *fixed, PointShapeDescriptor *constrained):
    LinePointDistanceConstraint(fixed, constrained, 0)
{
    line_fixed = fixed;
    point_constrained = constrained;
}

int LinePointCoincidentConstraint::getNumConstraints()
{
    return 2;
}

Eigen::VectorXd LinePointCoincidentConstraint::calculateConstraintValue(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed;
    Eigen::Matrix3d R_fixed;
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        R_fixed = Eigen::Quaterniond(vec(13), vec(10), vec(11), vec(12)).toRotationMatrix();
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    t = vec.segment(0,3);
    R = Eigen::Quaterniond(vec(6), vec(3), vec(4), vec(5)).toRotationMatrix();
    Eigen::Vector3d n1, n2;
    getOrthonormals(line_fixed->axis->vec, n1, n2);
    Eigen::VectorXd val;
    val.resize(2);
    val(0) = (n1.transpose())*(line_fixed->point->point-t-R*point_constrained->point);
    val(1) = (n2.transpose())*(line_fixed->point->point-t-R*point_constrained->point);
    return val;
}

Eigen::VectorXd LinePointCoincidentConstraint::calculateConstraintValueAA(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed;
    Eigen::Matrix3d R_fixed;
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        Eigen::Vector3d w_fixed = vec.segment(10,3);
        R_fixed = Eigen::AngleAxisd(vec(13), w_fixed);
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    t = vec.segment(0,3);
    Eigen::Vector3d w = vec.segment(3,3);
    R = Eigen::AngleAxisd(vec(6), w);
    Eigen::Vector3d n1, n2;
    getOrthonormals(line_fixed->axis->vec, n1, n2);
    Eigen::VectorXd val;
    val.resize(2);
    val(0) = (n1.transpose())*(line_fixed->point->point-t-R*point_constrained->point);
    val(1) = (n2.transpose())*(line_fixed->point->point-t-R*point_constrained->point);
    return val;
}

Eigen::MatrixXd LinePointCoincidentConstraint::calculateConstraintDerivativeAA(Eigen::VectorXd &vec)
{

}

Eigen::MatrixXd LinePointCoincidentConstraint::calculateConstraintDerivative(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed, w_fixed;
    Eigen::Matrix3d R_fixed;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        w_fixed = vec.segment(10,3);
        R_fixed = Eigen::AngleAxisd(vec(13), w_fixed);
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    Eigen::Vector3d n1, n2;
    getOrthonormals(line_fixed->axis->vec, n1, n2);
    Eigen::MatrixXd val;
    val.resize(2,7);
    val(0,0)=n1(0);
    val(0,1)=n1(1);
    val(0,2)=n1(2);
    val(0,3)=n1.transpose()*getdRx(vec.segment(3,4))*point_constrained->point;
    val(0,4)=n1.transpose()*getdRy(vec.segment(3,4))*point_constrained->point;
    val(0,5)=n1.transpose()*getdRz(vec.segment(3,4))*point_constrained->point;
    val(0,6)=n1.transpose()*getdRtheta(vec.segment(3,4))*point_constrained->point;
    val(1,0)=n2(0);
    val(1,1)=n2(1);
    val(1,2)=n2(2);
    val(1,3)=n2.transpose()*getdRx(vec.segment(3,4))*point_constrained->point;
    val(1,4)=n2.transpose()*getdRy(vec.segment(3,4))*point_constrained->point;
    val(1,5)=n2.transpose()*getdRz(vec.segment(3,4))*point_constrained->point;
    val(1,6)=n2.transpose()*getdRtheta(vec.segment(3,4))*point_constrained->point;
    return val;
}

Eigen::VectorXd LinePointCoincidentConstraint::getLowerBounds()
{
    Eigen::VectorXd lb;
    lb.resize(2);
    lb << 0,0;
    return lb;
}

Eigen::VectorXd LinePointCoincidentConstraint::getUpperBounds()
{
    Eigen::VectorXd ub;
    ub.resize(2);
    ub << 0,0;
    return ub;
}

LinePointDistanceMinMaxConstraint::LinePointDistanceMinMaxConstraint():
    OperationalPositionConstraint(std::string("line_point_min_max"))
{

}

LinePointDistanceMinMaxConstraint::LinePointDistanceMinMaxConstraint(LineShapeDescriptor *fixed, PointShapeDescriptor *constrained, double d_min, double d_max, bool maximize_d, bool minimize_d):
    OperationalPositionConstraint(std::string("line_point_distance_min_max")),
    line_fixed(fixed),
    point_constrained(constrained),
    distance_max(d_max),
    distance_min(d_min),
    maximize_distance(maximize_d),
    minimize_distance(minimize_d)
{
}

int LinePointDistanceMinMaxConstraint::getNumConstraints()
{
    return 1;
}

Eigen::VectorXd LinePointDistanceMinMaxConstraint::calculateConstraintValue(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed;
    Eigen::Matrix3d R_fixed;
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        R_fixed = Eigen::Quaterniond(vec(13), vec(10), vec(11), vec(12)).toRotationMatrix();
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    t = vec.segment(0,3);
    R = Eigen::Quaterniond(vec(6), vec(3), vec(4), vec(5)).toRotationMatrix();
    Eigen::Vector3d line_min_point;
    line_min_point = line_fixed->point->point + ((t+R*point_constrained->point - line_fixed->point->point).dot(line_fixed->axis->vec))*line_fixed->axis->vec;
    Eigen::VectorXd val;
    val.resize(1);
    val(0) = (line_min_point-t-R*point_constrained->point).transpose()*(line_min_point-t-R*point_constrained->point);
    return val;
}

Eigen::VectorXd LinePointDistanceMinMaxConstraint::calculateConstraintValueAA(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed;
    Eigen::Matrix3d R_fixed;
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        Eigen::Vector3d w_fixed = vec.segment(10,3);
        R_fixed = Eigen::AngleAxisd(vec(13), w_fixed);
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    t = vec.segment(0,3);
    Eigen::Vector3d w = vec.segment(3,3);
    R = Eigen::AngleAxisd(vec(6), w);
    Eigen::Vector3d line_min_point;
    line_min_point = line_fixed->point->point + ((t+R*point_constrained->point - line_fixed->point->point).dot(line_fixed->axis->vec))*line_fixed->axis->vec;
    Eigen::VectorXd val;
    val.resize(1);
    val(0) = (line_min_point-t-R*point_constrained->point).transpose()*(line_min_point-t-R*point_constrained->point);
    return val;
}

Eigen::MatrixXd LinePointDistanceMinMaxConstraint::calculateConstraintDerivativeAA(Eigen::VectorXd &vec)
{

}

Eigen::MatrixXd LinePointDistanceMinMaxConstraint::calculateConstraintDerivative(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed, w_fixed;
    Eigen::Matrix3d R_fixed;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        w_fixed = vec.segment(10,3);
        R_fixed = Eigen::AngleAxisd(vec(13), w_fixed);
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    Eigen::Vector3d t = vec.segment(0,3);
    Eigen::Vector3d w = vec.segment(3,3);
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(vec(6), w);
    Eigen::Vector3d line_min_point;
    line_min_point = line_fixed->point->point + ((t+R*point_constrained->point-line_fixed->point->point).dot(line_fixed->axis->vec))*line_fixed->axis->vec;
    Eigen::MatrixXd val;
    val.resize(1,7);
    val(0,0)=2*(line_min_point-t-R*point_constrained->point)(0);
    val(0,1)=2*(line_min_point-t-R*point_constrained->point)(1);
    val(0,2)=2*(line_min_point-t-R*point_constrained->point)(2);
    val(0,3)=2*(line_min_point-t-R*point_constrained->point).transpose()*getdRx(vec.segment(3,4))*point_constrained->point;
    val(0,4)=2*(line_min_point-t-R*point_constrained->point).transpose()*getdRy(vec.segment(3,4))*point_constrained->point;
    val(0,5)=2*(line_min_point-t-R*point_constrained->point).transpose()*getdRz(vec.segment(3,4))*point_constrained->point;
    val(0,6)=2*(line_min_point-t-R*point_constrained->point).transpose()*getdRtheta(vec.segment(3,4))*point_constrained->point;
    return val;
}

Eigen::VectorXd LinePointDistanceMinMaxConstraint::getLowerBounds()
{
    Eigen::VectorXd lb;
    lb.resize(2);
    lb << distance_min*distance_min,1.0;
    return lb;
}

Eigen::VectorXd LinePointDistanceMinMaxConstraint::getUpperBounds()
{
    Eigen::VectorXd ub;
    ub.resize(2);
    ub << distance_max*distance_max,1.0;
    return ub;
}

PlaneLineParallelConstraint::PlaneLineParallelConstraint(PlaneShapeDescriptor *fixed, LineShapeDescriptor *constrained):
    OperationalPositionConstraint("plane_line_parallel_constraint"),
    plane_fixed(fixed),
    line_constrained(constrained)
{

}

int PlaneLineParallelConstraint::getNumConstraints()
{
    return 2;
}

Eigen::VectorXd PlaneLineParallelConstraint::calculateConstraintValue(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed;
    Eigen::Matrix3d R_fixed;
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        R_fixed = Eigen::Quaterniond(vec(13), vec(10), vec(11), vec(12)).toRotationMatrix();
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    t = vec.segment(0,3);
    R = Eigen::Quaterniond(vec(6), vec(3), vec(4), vec(5)).toRotationMatrix();
    Eigen::VectorXd val;
    val.resize(2);
    val(0) = (R_fixed*plane_fixed->normal_direction->vec).dot((R_fixed*plane_fixed->point->point+t_fixed)-(R*line_constrained->point->point+t));
    val(1) = (R_fixed*plane_fixed->normal_direction->vec).dot(R*line_constrained->axis->vec);
    return val;
}

Eigen::VectorXd PlaneLineParallelConstraint::calculateConstraintValueAA(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed;
    Eigen::Matrix3d R_fixed;
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        Eigen::Vector3d w_fixed = vec.segment(10,3);
        R_fixed = Eigen::AngleAxisd(vec(13), w_fixed);
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    t = vec.segment(0,3);
    Eigen::Vector3d w = vec.segment(3,3);
    R = Eigen::AngleAxisd(vec(6), w);
    Eigen::VectorXd val;
    val.resize(2);
    val(0) = (R_fixed*plane_fixed->normal_direction->vec).dot((R_fixed*plane_fixed->point->point+t_fixed)-(R*line_constrained->point->point+t));
    val(1) = (R_fixed*plane_fixed->normal_direction->vec).dot(R*line_constrained->axis->vec);
    return val;
}

Eigen::MatrixXd PlaneLineParallelConstraint::calculateConstraintDerivativeAA(Eigen::VectorXd &vec)
{

}

Eigen::MatrixXd PlaneLineParallelConstraint::calculateConstraintDerivative(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed, w_fixed;
    Eigen::Matrix3d R_fixed;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        w_fixed = vec.segment(10,3);
        R_fixed = Eigen::AngleAxisd(vec(13), w_fixed);
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    Eigen::Vector3d w = vec.segment(3,3);
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(vec(6), w);
    Eigen::MatrixXd val;
    val.resize(2,7);
    val(0,0)=(R_fixed*plane_fixed->normal_direction->vec)(0);
    val(0,1)=(R_fixed*plane_fixed->normal_direction->vec)(1);
    val(0,2)=(R_fixed*plane_fixed->normal_direction->vec)(2);
    val(0,3)=(R_fixed*plane_fixed->normal_direction->vec).transpose()*getdRx(vec.segment(3,4))*line_constrained->point->point;
    val(0,4)=(R_fixed*plane_fixed->normal_direction->vec).transpose()*getdRy(vec.segment(3,4))*line_constrained->point->point;
    val(0,5)=(R_fixed*plane_fixed->normal_direction->vec).transpose()*getdRz(vec.segment(3,4))*line_constrained->point->point;
    val(0,6)=(R_fixed*plane_fixed->normal_direction->vec).transpose()*getdRtheta(vec.segment(3,4))*line_constrained->point->point;
    val(1,0)=0;
    val(1,1)=0;
    val(1,2)=0;
    val(1,3)=(R_fixed*plane_fixed->normal_direction->vec).transpose()*getdRx(vec.segment(3,4))*line_constrained->axis->vec;
    val(1,4)=(R_fixed*plane_fixed->normal_direction->vec).transpose()*getdRy(vec.segment(3,4))*line_constrained->axis->vec;
    val(1,5)=(R_fixed*plane_fixed->normal_direction->vec).transpose()*getdRz(vec.segment(3,4))*line_constrained->axis->vec;
    val(1,6)=(R_fixed*plane_fixed->normal_direction->vec).transpose()*getdRtheta(vec.segment(3,4))*line_constrained->axis->vec;
    return val;
}

Eigen::VectorXd PlaneLineParallelConstraint::getLowerBounds()
{
    Eigen::VectorXd lb;
    lb.resize(2);
    lb << 0,0;
    return lb;
}

Eigen::VectorXd PlaneLineParallelConstraint::getUpperBounds()
{
    Eigen::VectorXd ub;
    ub.resize(2);
    ub << 0,0;
    return ub;
}

PointPointDistanceMinMaxConstraint::PointPointDistanceMinMaxConstraint(PointShapeDescriptor *fixed, PointShapeDescriptor *constrained, double d_min, double d_max):
    OperationalPositionConstraint("point_point_coincident_constraint"),
    point_fixed(fixed),
    point_constrained(constrained),
    distance_min(d_min),
    distance_max(d_max)
{

}

int PointPointDistanceMinMaxConstraint::getNumConstraints()
{
    return 1;
}

Eigen::VectorXd PointPointDistanceMinMaxConstraint::calculateConstraintValue(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed;
    Eigen::Matrix3d R_fixed;
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        R_fixed = Eigen::Quaterniond(vec(13), vec(10), vec(11), vec(12)).toRotationMatrix();
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    t = vec.segment(0,3);
    R = Eigen::Quaterniond(vec(6), vec(3), vec(4), vec(5)).toRotationMatrix();
    Eigen::VectorXd val;
    val.resize(1);
    Eigen::Vector3d d = ((R*point_constrained->point+t)-(R_fixed*point_fixed->point+t_fixed));
    val(0) = d.transpose()*d;
    return val;
}

Eigen::VectorXd PointPointDistanceMinMaxConstraint::calculateConstraintValueAA(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed;
    Eigen::Matrix3d R_fixed;
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        Eigen::Vector3d w_fixed = vec.segment(10,3);
        R_fixed = Eigen::AngleAxisd(vec(13), w_fixed);
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    t = vec.segment(0,3);
    Eigen::Vector3d w = vec.segment(3,3);
    R = Eigen::AngleAxisd(vec(6), w);
    Eigen::VectorXd val;
    val.resize(1);
    Eigen::Vector3d d = ((R*point_constrained->point+t)-(R_fixed*point_fixed->point+t_fixed));
    val(0) = d.transpose()*d;
    return val;
}

Eigen::MatrixXd PointPointDistanceMinMaxConstraint::calculateConstraintDerivativeAA(Eigen::VectorXd &vec)
{

}

Eigen::MatrixXd PointPointDistanceMinMaxConstraint::calculateConstraintDerivative(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed, w_fixed;
    Eigen::Matrix3d R_fixed;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        w_fixed = vec.segment(10,3);
        R_fixed = Eigen::AngleAxisd(vec(13), w_fixed);
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    Eigen::Vector3d t = vec.segment(0,3);
    Eigen::Vector3d w = vec.segment(3,3);
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(vec(6), w);
    Eigen::MatrixXd val;
    Eigen::Vector3d d = ((R*point_constrained->point+t)-(R_fixed*point_fixed->point+t_fixed));
    val.resize(1,7);
    val(0,0)=2*d(0);
    val(0,1)=2*d(1);
    val(0,2)=2*d(2);
    val(0,3)=2*d.transpose()*(getdRx(vec.segment(3,4))*point_constrained->point);
    val(0,4)=2*d.transpose()*(getdRy(vec.segment(3,4))*point_constrained->point);
    val(0,5)=2*d.transpose()*(getdRz(vec.segment(3,4))*point_constrained->point);
    val(0,6)=2*d.transpose()*(getdRtheta(vec.segment(3,4))*point_constrained->point);
    return val;
}

Eigen::VectorXd PointPointDistanceMinMaxConstraint::getLowerBounds()
{
    Eigen::VectorXd lb;
    lb.resize(1);
    lb << distance_min;
    return lb;
}

Eigen::VectorXd PointPointDistanceMinMaxConstraint::getUpperBounds()
{
    Eigen::VectorXd ub;
    ub.resize(1);
    ub << distance_max;
    return ub;
}

PointLineCoincidentConstraint::PointLineCoincidentConstraint(PointShapeDescriptor *fixed, LineShapeDescriptor *constrained):
    OperationalPositionConstraint("point_line_coincident_constraint"),
    point_fixed(fixed),
    line_constrained(constrained)
{
}

int PointLineCoincidentConstraint::getNumConstraints()
{
    return 3;
}

Eigen::VectorXd PointLineCoincidentConstraint::calculateConstraintValue(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed;
    Eigen::Matrix3d R_fixed;
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        R_fixed = Eigen::Quaterniond(vec(13), vec(10), vec(11), vec(12)).toRotationMatrix();
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    t = vec.segment(0,3);
    R = Eigen::Quaterniond(vec(6), vec(3), vec(4), vec(5)).toRotationMatrix();
    Eigen::Vector3d line_min_point;
    Eigen::Vector3d line_T_point, line_T_axis;
    line_T_point = R*line_constrained->point->point+t;
    line_T_axis = R*line_constrained->axis->vec;
    line_min_point = line_T_point + (((R_fixed*point_fixed->point+t_fixed)-line_T_point).dot(line_T_axis))*(line_T_axis);
    Eigen::VectorXd val;
    val.resize(3);
    val = line_min_point-(R_fixed*point_fixed->point+t_fixed);
    return val;
}

Eigen::VectorXd PointLineCoincidentConstraint::calculateConstraintValueAA(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed;
    Eigen::Matrix3d R_fixed;
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        Eigen::Vector3d w_fixed = vec.segment(10,3);
        R_fixed = Eigen::AngleAxisd(vec(13), w_fixed);
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    t = vec.segment(0,3);
    Eigen::Vector3d w = vec.segment(3,3);
    R = Eigen::AngleAxisd(vec(6), w);
    Eigen::Vector3d line_min_point;
    Eigen::Vector3d line_T_point, line_T_axis;
    line_T_point = R*line_constrained->point->point+t;
    line_T_axis = R*line_constrained->axis->vec;
    line_min_point = line_T_point + (((R_fixed*point_fixed->point+t_fixed)-line_T_point).dot(line_T_axis))*(line_T_axis);
    Eigen::VectorXd val;
    val.resize(3);
    val = line_min_point-(R_fixed*point_fixed->point+t_fixed);
    return val;
}

Eigen::MatrixXd PointLineCoincidentConstraint::calculateConstraintDerivativeAA(Eigen::VectorXd &vec)
{

}


Eigen::MatrixXd PointLineCoincidentConstraint::calculateConstraintDerivative(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed, w_fixed;
    Eigen::Matrix3d R_fixed;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        w_fixed = vec.segment(10,3);
        R_fixed = Eigen::AngleAxisd(vec(13), w_fixed);
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    Eigen::Vector3d t = vec.segment(0,3);
    Eigen::Vector3d w = vec.segment(3,3);
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(vec(6), w);
    Eigen::MatrixXd val;
    val.resize(3,7);
    //@TODO: symbolic derivative
    return val;
}

Eigen::VectorXd PointLineCoincidentConstraint::getLowerBounds()
{
    Eigen::VectorXd lb;
    lb.resize(3);
    lb << 0,0,0;
    return lb;
}

Eigen::VectorXd PointLineCoincidentConstraint::getUpperBounds()
{
    Eigen::VectorXd ub;
    ub.resize(3);
    ub << 0,0,0;
    return ub;
}

PointLineDistanceMinMaxConstraint::PointLineDistanceMinMaxConstraint(PointShapeDescriptor *fixed, LineShapeDescriptor *constrained, double d_min, double d_max, bool maximize_d, bool minimize_d):
    OperationalPositionConstraint("point_line_distance_min_max_constraint"),
    point_fixed(fixed),
    line_constrained(constrained),
    distance_min(d_min),
    distance_max(d_max),
    minimize_distance(minimize_d),
    maximize_distance(maximize_d)
{

}

int PointLineDistanceMinMaxConstraint::getNumConstraints()
{
    return 1;
}

Eigen::VectorXd PointLineDistanceMinMaxConstraint::calculateConstraintValue(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed;
    Eigen::Matrix3d R_fixed;
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        R_fixed = Eigen::Quaterniond(vec(13), vec(10), vec(11), vec(12)).toRotationMatrix();
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    t = vec.segment(0,3);
    R = Eigen::Quaterniond(vec(6), vec(3), vec(4), vec(5)).toRotationMatrix();
    Eigen::Vector3d line_min_point;
    Eigen::Vector3d line_T_point, line_T_axis;
    line_T_point = R*line_constrained->point->point+t;
    line_T_axis = R*line_constrained->axis->vec;
    line_min_point = line_T_point + (((R_fixed*point_fixed->point+t_fixed)-line_T_point).dot(line_T_axis))*(line_T_axis);
    Eigen::VectorXd val;
    val.resize(1);
    val(0) = (line_min_point-(R_fixed*point_fixed->point+t_fixed)).transpose()*(line_min_point-(R_fixed*point_fixed->point+t_fixed));
    return val;
}

Eigen::VectorXd PointLineDistanceMinMaxConstraint::calculateConstraintValueAA(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed;
    Eigen::Matrix3d R_fixed;
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        Eigen::Vector3d w_fixed = vec.segment(10,3);
        R_fixed = Eigen::AngleAxisd(vec(13), w_fixed);
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    t = vec.segment(0,3);
    Eigen::Vector3d w = vec.segment(3,3);
    R = Eigen::AngleAxisd(vec(6), w);
    Eigen::Vector3d line_min_point;
    Eigen::Vector3d line_T_point, line_T_axis;
    line_T_point = R*line_constrained->point->point+t;
    line_T_axis = R*line_constrained->axis->vec;
    line_min_point = line_T_point + (((R_fixed*point_fixed->point+t_fixed)-line_T_point).dot(line_T_axis))*(line_T_axis);
    Eigen::VectorXd val;
    val.resize(1);
    val(0) = (line_min_point-(R_fixed*point_fixed->point+t_fixed)).transpose()*(line_min_point-(R_fixed*point_fixed->point+t_fixed));
    return val;
}

Eigen::MatrixXd PointLineDistanceMinMaxConstraint::calculateConstraintDerivativeAA(Eigen::VectorXd &vec)
{

}

Eigen::MatrixXd PointLineDistanceMinMaxConstraint::calculateConstraintDerivative(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed, w_fixed;
    Eigen::Matrix3d R_fixed;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        w_fixed = vec.segment(10,3);
        R_fixed = Eigen::AngleAxisd(vec(13), w_fixed);
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    Eigen::Vector3d t = vec.segment(0,3);
    Eigen::Vector3d w = vec.segment(3,3);
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(vec(6), w);
    Eigen::MatrixXd val;
    val.resize(1,7);
    //@TODO: symbolic derivative
    return val;
}

Eigen::VectorXd PointLineDistanceMinMaxConstraint::getLowerBounds()
{
    Eigen::VectorXd lb;
    lb.resize(1);
    lb << distance_min*distance_min;
    return lb;
}

Eigen::VectorXd PointLineDistanceMinMaxConstraint::getUpperBounds()
{
    Eigen::VectorXd ub;
    ub.resize(1);
    ub << distance_max*distance_max;
    return ub;
}

ConeConeParallelDistanceMinMaxConstraint::ConeConeParallelDistanceMinMaxConstraint(ConeShapeDescriptor *fixed, ConeShapeDescriptor *constrained, double d_min, double d_max):
    OperationalPositionConstraint("cone_cone_parallel_distance_minx_max"),
    cone_fixed(fixed),
    cone_constrained(constrained),
    distance_max(d_max),
    distance_min(d_min)
{

}

int ConeConeParallelDistanceMinMaxConstraint::getNumConstraints()
{
    return 2;
}

Eigen::VectorXd ConeConeParallelDistanceMinMaxConstraint::calculateConstraintValue(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed;
    Eigen::Matrix3d R_fixed;
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        R_fixed = Eigen::Quaterniond(vec(13), vec(10), vec(11), vec(12)).toRotationMatrix();
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    t = vec.segment(0,3);
    R = Eigen::Quaterniond(vec(6), vec(3), vec(4), vec(5)).toRotationMatrix();
    Eigen::VectorXd val;
    val.resize(2);
    Eigen::Vector3d p2_t = R*cone_constrained->center->point+t;
    Eigen::Vector3d cone_min_point = (R_fixed*cone_fixed->center->point+t_fixed) + ((R_fixed*cone_fixed->axis->vec).dot(p2_t-(R_fixed*cone_fixed->center->point+t_fixed)))*(R_fixed*cone_fixed->axis->vec);
    Eigen::Vector3d d = p2_t - cone_min_point;
    val(0) = d.transpose()*d;
    val(1) = (R*cone_constrained->axis->vec).dot((R_fixed*cone_fixed->axis->vec));

    return val;
}

Eigen::VectorXd ConeConeParallelDistanceMinMaxConstraint::calculateConstraintValueAA(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed;
    Eigen::Matrix3d R_fixed;
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        Eigen::Vector3d w_fixed = vec.segment(10,3);
        R_fixed = Eigen::AngleAxisd(vec(13), w_fixed);
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    t = vec.segment(0,3);
    Eigen::Vector3d w = vec.segment(3,3);
    R = Eigen::AngleAxisd(vec(6), w);
    Eigen::VectorXd val;
    val.resize(2);
    Eigen::Vector3d p2_t = R*cone_constrained->center->point+t;
    Eigen::Vector3d cone_min_point = (R_fixed*cone_fixed->center->point+t_fixed) + ((R_fixed*cone_fixed->axis->vec).dot(p2_t-(R_fixed*cone_fixed->center->point+t_fixed)))*(R_fixed*cone_fixed->axis->vec);
    Eigen::Vector3d d = p2_t - cone_min_point;
    val(0) = d.transpose()*d;
    val(1) = (R*cone_constrained->axis->vec).dot((R_fixed*cone_fixed->axis->vec));

    return val;
}

Eigen::MatrixXd ConeConeParallelDistanceMinMaxConstraint::calculateConstraintDerivativeAA(Eigen::VectorXd &vec)
{

}

Eigen::MatrixXd ConeConeParallelDistanceMinMaxConstraint::calculateConstraintDerivative(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed, w_fixed;
    Eigen::Matrix3d R_fixed;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        w_fixed = vec.segment(10,3);
        R_fixed = Eigen::AngleAxisd(vec(13), w_fixed);
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    Eigen::Vector3d t = vec.segment(0,3);
    Eigen::Vector3d w = vec.segment(3,3);
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(vec(6), w);
    Eigen::Vector3d p2_t = R*cone_constrained->center->point+t;
    Eigen::Vector3d cone_min_point = (R_fixed*cone_fixed->center->point+t_fixed) + ((R_fixed*cone_fixed->axis->vec).dot(p2_t-(R_fixed*cone_fixed->center->point+t_fixed)))*(R_fixed*cone_fixed->axis->vec);
    Eigen::Vector3d d = p2_t - cone_min_point;

    Eigen::VectorXd val;
    val.resize(2,7);
    val(0,0) = 2*d(0);
    val(0,1) = 2*d(1);
    val(0,2) = 2*d(2);
    val(0,3) = 2*d.transpose()*getdRx(vec.segment(3,4))*cone_constrained->center->point;
    val(0,4) = 2*d.transpose()*getdRy(vec.segment(3,4))*cone_constrained->center->point;
    val(0,5) = 2*d.transpose()*getdRz(vec.segment(3,4))*cone_constrained->center->point;
    val(0,6) = 2*d.transpose()*getdRtheta(vec.segment(3,4))*cone_constrained->center->point;
    val(1,0) = 0;
    val(1,1) = 0;
    val(1,2) = 0;
    val(1,3) = (R_fixed*cone_fixed->axis->vec).transpose()*getdRx(vec.segment(3,4))*cone_constrained->axis->vec;
    val(1,4) = (R_fixed*cone_fixed->axis->vec).transpose()*getdRy(vec.segment(3,4))*cone_constrained->axis->vec;
    val(1,5) = (R_fixed*cone_fixed->axis->vec).transpose()*getdRz(vec.segment(3,4))*cone_constrained->axis->vec;
    val(1,6) = (R_fixed*cone_fixed->axis->vec).transpose()*getdRtheta(vec.segment(3,4))*cone_constrained->axis->vec;

    return val;
}

Eigen::VectorXd ConeConeParallelDistanceMinMaxConstraint::getLowerBounds()
{
    Eigen::VectorXd lb;
    lb.resize(2);
    lb << distance_min*distance_min,1.0;
    return lb;
}

Eigen::VectorXd ConeConeParallelDistanceMinMaxConstraint::getUpperBounds()
{
    Eigen::VectorXd ub;
    ub.resize(2);
    ub << distance_max*distance_max,1.0;
    return ub;
}

LineLineParallelDistanceMinMaxConstraint::LineLineParallelDistanceMinMaxConstraint(LineShapeDescriptor *fixed, LineShapeDescriptor *constrained, double d_min, double d_max, bool maximize_d, bool minimize_d):
    OperationalPositionConstraint("line_line_parallel_distance_minx_max"),
    line_fixed(fixed),
    line_constrained(constrained),
    distance_max(d_max),
    distance_min(d_min)
{
    constraint_name = "line_line_parallel_distance_min_max";
}

int LineLineParallelDistanceMinMaxConstraint::getNumConstraints()
{
    return 2;
}

Eigen::VectorXd LineLineParallelDistanceMinMaxConstraint::calculateConstraintValue(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed;
    Eigen::Matrix3d R_fixed;
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        R_fixed = Eigen::Quaterniond(vec(13), vec(10), vec(11), vec(12)).toRotationMatrix();
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    t = vec.segment(0,3);
    R = Eigen::Quaterniond(vec(6), vec(3), vec(4), vec(5)).toRotationMatrix();
    Eigen::VectorXd val;
    val.resize(2);
    Eigen::Vector3d p2_t = R*line_constrained->point->point+t;
    Eigen::Vector3d line_min_point = (R_fixed*line_fixed->point->point+t_fixed) + ((R_fixed*line_fixed->axis->vec).dot(p2_t-(R_fixed*line_fixed->point->point+t_fixed)))*(R_fixed*line_fixed->axis->vec);
    Eigen::Vector3d d = p2_t - line_min_point;
    val(0) = d.transpose()*d;
    val(1) = (R*line_constrained->axis->vec).dot((R_fixed*line_fixed->axis->vec));

    return val;
}

Eigen::VectorXd LineLineParallelDistanceMinMaxConstraint::calculateConstraintValueAA(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed;
    Eigen::Matrix3d R_fixed;
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        Eigen::Vector3d w_fixed = vec.segment(10,3);
        R_fixed = Eigen::AngleAxisd(vec(13), w_fixed);
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    t = vec.segment(0,3);
    Eigen::Vector3d w = vec.segment(3,3);
    R = Eigen::AngleAxisd(vec(6), w);
    Eigen::VectorXd val;
    val.resize(2);
    Eigen::Vector3d p2_t = R*line_constrained->point->point+t;
    Eigen::Vector3d line_min_point = (R_fixed*line_fixed->point->point+t_fixed) + ((R_fixed*line_fixed->axis->vec).dot(p2_t-(R_fixed*line_fixed->point->point+t_fixed)))*(R_fixed*line_fixed->axis->vec);
    Eigen::Vector3d d = p2_t - line_min_point;
    val(0) = d.transpose()*d;
    val(1) = (R*line_constrained->axis->vec).dot((R_fixed*line_fixed->axis->vec));

    return val;
}

Eigen::MatrixXd LineLineParallelDistanceMinMaxConstraint::calculateConstraintDerivativeAA(Eigen::VectorXd &vec)
{

}

Eigen::MatrixXd LineLineParallelDistanceMinMaxConstraint::calculateConstraintDerivative(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed, w_fixed;
    Eigen::Matrix3d R_fixed;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        w_fixed = vec.segment(10,3);
        R_fixed = Eigen::AngleAxisd(vec(13), w_fixed);
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    Eigen::Vector3d t = vec.segment(0,3);
    Eigen::Vector3d w = vec.segment(3,3);
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(vec(6), w);
    Eigen::Vector3d p2_t = R*line_constrained->point->point+t;
    Eigen::Vector3d line_min_point = (R_fixed*line_fixed->point->point+t_fixed) + ((R_fixed*line_fixed->axis->vec).dot(p2_t-(R_fixed*line_fixed->point->point+t_fixed)))*(R_fixed*line_fixed->axis->vec);
    Eigen::Vector3d d = p2_t - line_min_point;

    Eigen::VectorXd val;
    val.resize(2,7);
    val(0,0) = 2*d(0);
    val(0,1) = 2*d(1);
    val(0,2) = 2*d(2);
    val(0,3) = 2*d.transpose()*getdRx(vec.segment(3,4))*line_constrained->point->point;
    val(0,4) = 2*d.transpose()*getdRy(vec.segment(3,4))*line_constrained->point->point;
    val(0,5) = 2*d.transpose()*getdRz(vec.segment(3,4))*line_constrained->point->point;
    val(0,6) = 2*d.transpose()*getdRtheta(vec.segment(3,4))*line_constrained->point->point;
    val(1,0) = 0;
    val(1,1) = 0;
    val(1,2) = 0;
    val(1,3) = (R_fixed*line_fixed->axis->vec).transpose()*getdRx(vec.segment(3,4))*line_constrained->axis->vec;
    val(1,4) = (R_fixed*line_fixed->axis->vec).transpose()*getdRy(vec.segment(3,4))*line_constrained->axis->vec;
    val(1,5) = (R_fixed*line_fixed->axis->vec).transpose()*getdRz(vec.segment(3,4))*line_constrained->axis->vec;
    val(1,6) = (R_fixed*line_fixed->axis->vec).transpose()*getdRtheta(vec.segment(3,4))*line_constrained->axis->vec;

    return val;

}

Eigen::VectorXd LineLineParallelDistanceMinMaxConstraint::getLowerBounds()
{
    Eigen::VectorXd lb;
    lb.resize(2);
    lb << distance_min*distance_min,1.0;
    return lb;
}

Eigen::VectorXd LineLineParallelDistanceMinMaxConstraint::getUpperBounds()
{
    Eigen::VectorXd ub;
    ub.resize(2);
    ub << distance_max*distance_max,1.0;
    return ub;
}

LineLineCoincidentConstraint::LineLineCoincidentConstraint(LineShapeDescriptor *fixed, LineShapeDescriptor *constrained):
    LineLineParallelDistanceConstraint(fixed, constrained, 0)
{
    constraint_name = "line_line_coincident";
    this->line_fixed = fixed;
    this->line_constrained = constrained;
}

PointPointDistanceConstraint::PointPointDistanceConstraint(PointShapeDescriptor *fixed, PointShapeDescriptor *constrained, double d):
    PointPointDistanceMinMaxConstraint(fixed, constrained, d, d), distance(d)
{
    constraint_name = "point_point_distance";
}

PointPointCoincidentConstraint::PointPointCoincidentConstraint(PointShapeDescriptor *fixed, PointShapeDescriptor *constrained):
    PointPointDistanceMinMaxConstraint(fixed, constrained, 0, 0)
{
    constraint_name = "point_point_coincident";
    point_fixed = fixed;
    point_constrained = constrained;
}

LineLineParallelConstraint::LineLineParallelConstraint(LineShapeDescriptor *fixed, LineShapeDescriptor *constrained):
    OperationalPositionConstraint("line_line_parallel"),
    line_fixed(fixed),
    line_constrained(constrained)
{
}

int LineLineParallelConstraint::getNumConstraints()
{
    return 1;
}

Eigen::VectorXd LineLineParallelConstraint::calculateConstraintValue(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed;
    Eigen::Matrix3d R_fixed;
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        R_fixed = Eigen::Quaterniond(vec(13), vec(10), vec(11), vec(12)).toRotationMatrix();
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    t = vec.segment(0,3);
    R = Eigen::Quaterniond(vec(6), vec(3), vec(4), vec(5)).toRotationMatrix();
    Eigen::VectorXd val;
    val.resize(1);
    val(0) = (R*line_constrained->axis->vec).dot((R_fixed*line_fixed->axis->vec));
    return val;
}

Eigen::VectorXd LineLineParallelConstraint::calculateConstraintValueAA(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed;
    Eigen::Matrix3d R_fixed;
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        Eigen::Vector3d w_fixed = vec.segment(10,3);
        R_fixed = Eigen::AngleAxisd(vec(13), w_fixed);
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    t = vec.segment(0,3);
    Eigen::Vector3d w = vec.segment(3,3);
    R = Eigen::AngleAxisd(vec(6), w);
    Eigen::VectorXd val;
    val.resize(1);
    val(0) = (R*line_constrained->axis->vec).dot((R_fixed*line_fixed->axis->vec));
    return val;
}

Eigen::MatrixXd LineLineParallelConstraint::calculateConstraintDerivativeAA(Eigen::VectorXd &vec)
{

}

Eigen::MatrixXd LineLineParallelConstraint::calculateConstraintDerivative(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed, w_fixed;
    Eigen::Matrix3d R_fixed;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        w_fixed = vec.segment(10,3);
        R_fixed = Eigen::AngleAxisd(vec(13), w_fixed);
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    Eigen::Vector3d t = vec.segment(0,3);
    Eigen::Vector3d w = vec.segment(3,3);

    Eigen::VectorXd val;
    val.resize(1,7);
    val(0,0) = 0;
    val(0,1) = 0;
    val(0,2) = 0;
    val(0,3) = (R_fixed*line_fixed->axis->vec).transpose()*getdRx(vec.segment(3,4))*line_constrained->axis->vec;
    val(0,4) = (R_fixed*line_fixed->axis->vec).transpose()*getdRy(vec.segment(3,4))*line_constrained->axis->vec;
    val(0,5) = (R_fixed*line_fixed->axis->vec).transpose()*getdRz(vec.segment(3,4))*line_constrained->axis->vec;
    val(0,6) = (R_fixed*line_fixed->axis->vec).transpose()*getdRtheta(vec.segment(3,4))*line_constrained->axis->vec;

    return val;

}

Eigen::VectorXd LineLineParallelConstraint::getLowerBounds()
{
    Eigen::VectorXd lb;
    lb.resize(1);
    lb << 1.0;
    return lb;
}

Eigen::VectorXd LineLineParallelConstraint::getUpperBounds()
{
    Eigen::VectorXd ub;
    ub.resize(1);
    ub << 1.0;
    return ub;
}


PlanePlaneSymmetryConstraint::PlanePlaneSymmetryConstraint(PlaneShapeDescriptor *fixed1, PlaneShapeDescriptor *fixed2, PlaneShapeDescriptor *constrained1, PlaneShapeDescriptor *constrained2):
    OperationalPositionConstraint(std::string("plane_plane_symmetry")),
    plane_fixed1(fixed1),
    plane_fixed2(fixed2),
    plane_constrained1(constrained1),
    plane_constrained2(constrained2)
{

}

int PlanePlaneSymmetryConstraint::getNumConstraints()
{
    return 3;
}

Eigen::VectorXd PlanePlaneSymmetryConstraint::calculateConstraintValue(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed;
    Eigen::Matrix3d R_fixed;
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        R_fixed = Eigen::Quaterniond(vec(13), vec(10), vec(11), vec(12)).toRotationMatrix();
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    t = vec.segment(0,3);
    R = Eigen::Quaterniond(vec(6), vec(3), vec(4), vec(5)).toRotationMatrix();
    Eigen::VectorXd val;
    val.resize(3);
    val(0) = (R*plane_constrained1->point->point+t-(R_fixed*plane_fixed1->point->point+t_fixed)).dot((R_fixed*plane_fixed1->normal_direction->vec))
            -(R*plane_constrained2->point->point+t-(R_fixed*plane_fixed2->point->point+t_fixed)).dot((R_fixed*plane_fixed2->normal_direction->vec));
    val(1) = (R*plane_constrained1->normal_direction->vec).dot((R_fixed*plane_fixed1->normal_direction->vec));
    val(2) = (R*plane_constrained2->normal_direction->vec).dot((R_fixed*plane_fixed2->normal_direction->vec));
    return val;
}

Eigen::VectorXd PlanePlaneSymmetryConstraint::calculateConstraintValueAA(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed;
    Eigen::Matrix3d R_fixed;
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        Eigen::Vector3d w_fixed = vec.segment(10,3);
        R_fixed = Eigen::AngleAxisd(vec(13), w_fixed);
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    t = vec.segment(0,3);
    Eigen::Vector3d w = vec.segment(3,3);
    R = Eigen::AngleAxisd(vec(6), w);
    Eigen::VectorXd val;
    val.resize(3);
    val(0) = (R*plane_constrained1->point->point+t-(R_fixed*plane_fixed1->point->point+t_fixed)).dot((R_fixed*plane_fixed1->normal_direction->vec))
            -(R*plane_constrained2->point->point+t-(R_fixed*plane_fixed2->point->point+t_fixed)).dot((R_fixed*plane_fixed2->normal_direction->vec));
    val(1) = (R*plane_constrained1->normal_direction->vec).dot((R_fixed*plane_fixed1->normal_direction->vec));
    val(2) = (R*plane_constrained2->normal_direction->vec).dot((R_fixed*plane_fixed2->normal_direction->vec));
    return val;
}

Eigen::MatrixXd PlanePlaneSymmetryConstraint::calculateConstraintDerivativeAA(Eigen::VectorXd &vec)
{

}

Eigen::MatrixXd PlanePlaneSymmetryConstraint::calculateConstraintDerivative(Eigen::VectorXd &vec)
{
    Eigen::Vector3d t_fixed, w_fixed;
    Eigen::Matrix3d R_fixed;
    if(vec.size() > 7)
    {
        t_fixed = vec.segment(7,3);
        w_fixed = vec.segment(10,3);
        R_fixed = Eigen::AngleAxisd(vec(13), w_fixed);
    }
    else
    {
        t_fixed.setZero();
        R_fixed.setIdentity();
    }
    Eigen::Vector4d axisAngle = vec.segment(3,4);
    Eigen::MatrixXd val;
    val.resize(3,7);
    val(0,0)=(R_fixed*plane_fixed1->normal_direction->vec)(0);
    val(0,1)=(R_fixed*plane_fixed1->normal_direction->vec)(1);
    val(0,2)=(R_fixed*plane_fixed1->normal_direction->vec)(2);
    val(0,3)=(R_fixed*plane_fixed1->normal_direction->vec).transpose()*getdRx(axisAngle)*plane_constrained1->point->point;
    val(0,4)=(R_fixed*plane_fixed1->normal_direction->vec).transpose()*getdRy(axisAngle)*plane_constrained1->point->point;
    val(0,5)=(R_fixed*plane_fixed1->normal_direction->vec).transpose()*getdRz(axisAngle)*plane_constrained1->point->point;
    val(0,6)=(R_fixed*plane_fixed1->normal_direction->vec).transpose()*getdRtheta(axisAngle)*plane_constrained1->point->point;
    val(0,0)-=(R_fixed*plane_fixed2->normal_direction->vec)(0);
    val(0,1)-=(R_fixed*plane_fixed2->normal_direction->vec)(1);
    val(0,2)-=(R_fixed*plane_fixed2->normal_direction->vec)(2);
    val(0,3)-=(R_fixed*plane_fixed2->normal_direction->vec).transpose()*getdRx(axisAngle)*plane_constrained2->point->point;
    val(0,4)-=(R_fixed*plane_fixed2->normal_direction->vec).transpose()*getdRy(axisAngle)*plane_constrained2->point->point;
    val(0,5)-=(R_fixed*plane_fixed2->normal_direction->vec).transpose()*getdRz(axisAngle)*plane_constrained2->point->point;
    val(0,6)-=(R_fixed*plane_fixed2->normal_direction->vec).transpose()*getdRtheta(axisAngle)*plane_constrained2->point->point;
    val(1,0)=0;
    val(1,1)=0;
    val(1,2)=0;
    val(1,3)=(R_fixed*plane_fixed1->normal_direction->vec).transpose()*getdRx(axisAngle)*plane_constrained1->normal_direction->vec;
    val(1,4)=(R_fixed*plane_fixed1->normal_direction->vec).transpose()*getdRy(axisAngle)*plane_constrained1->normal_direction->vec;
    val(1,5)=(R_fixed*plane_fixed1->normal_direction->vec).transpose()*getdRz(axisAngle)*plane_constrained1->normal_direction->vec;
    val(1,6)=(R_fixed*plane_fixed1->normal_direction->vec).transpose()*getdRtheta(axisAngle)*plane_constrained1->normal_direction->vec;
    val(1,0)=0;
    val(1,1)=0;
    val(1,2)=0;
    val(1,3)=(R_fixed*plane_fixed2->normal_direction->vec).transpose()*getdRx(axisAngle)*plane_constrained2->normal_direction->vec;
    val(1,4)=(R_fixed*plane_fixed2->normal_direction->vec).transpose()*getdRy(axisAngle)*plane_constrained2->normal_direction->vec;
    val(1,5)=(R_fixed*plane_fixed2->normal_direction->vec).transpose()*getdRz(axisAngle)*plane_constrained2->normal_direction->vec;
    val(1,6)=(R_fixed*plane_fixed2->normal_direction->vec).transpose()*getdRtheta(axisAngle)*plane_constrained2->normal_direction->vec;
    return val;
}

Eigen::VectorXd PlanePlaneSymmetryConstraint::getLowerBounds()
{
    Eigen::VectorXd lb;
    lb.resize(3);
    lb << 0, 1.0, 1.0;
    return lb;
}

Eigen::VectorXd PlanePlaneSymmetryConstraint::getUpperBounds()
{
    Eigen::VectorXd ub;
    ub.resize(3);
    ub << 0, 1.0, 1.0;
    return ub;
}



VectorVectorAngleConstraint::VectorVectorAngleConstraint(VectorShapeDescriptor *fixed, VectorShapeDescriptor *constrained, double angle):
    OperationalPositionConstraint("vector_vector_angle"),
    vector_fixed(fixed),
    vector_constrained(constrained),
    angle(angle)
{
}

//NODO
int VectorVectorAngleConstraint::getNumConstraints()
{

}

//NODO
Eigen::VectorXd VectorVectorAngleConstraint::calculateConstraintValue(Eigen::VectorXd &vec)
{

}

//NODO
Eigen::MatrixXd VectorVectorAngleConstraint::calculateConstraintDerivative(Eigen::VectorXd &vec)
{

}

Eigen::VectorXd VectorVectorAngleConstraint::calculateConstraintValueAA(Eigen::VectorXd &vec)
{

}

Eigen::MatrixXd VectorVectorAngleConstraint::calculateConstraintDerivativeAA(Eigen::VectorXd &vec)
{

}

//NODO
Eigen::VectorXd VectorVectorAngleConstraint::getLowerBounds()
{

}

//NODO
Eigen::VectorXd VectorVectorAngleConstraint::getUpperBounds()
{

}


VectorVectorCoincidentConstraint::VectorVectorCoincidentConstraint(VectorShapeDescriptor *fixed, VectorShapeDescriptor *constrained):
    VectorVectorAngleConstraint(fixed, constrained, 0)
{
    constraint_name = "vector_vector_coincident";
}


LineLineAngleConstraint::LineLineAngleConstraint(LineShapeDescriptor *fixed, LineShapeDescriptor *constrained, double angle):
    OperationalPositionConstraint("line_line_angle"),
    line_fixed(fixed), line_constrained(constrained), angle(angle)
{
}

//NODO
int LineLineAngleConstraint::getNumConstraints()
{

}

//NODO
Eigen::VectorXd LineLineAngleConstraint::calculateConstraintValue(Eigen::VectorXd &vec)
{

}

//NODO
Eigen::MatrixXd LineLineAngleConstraint::calculateConstraintDerivative(Eigen::VectorXd &vec)
{

}

//NODO
Eigen::VectorXd LineLineAngleConstraint::calculateConstraintValueAA(Eigen::VectorXd &vec)
{

}

Eigen::MatrixXd LineLineAngleConstraint::calculateConstraintDerivativeAA(Eigen::VectorXd &vec)
{

}

//NODO
Eigen::VectorXd LineLineAngleConstraint::getLowerBounds()
{

}

//NODO
Eigen::VectorXd LineLineAngleConstraint::getUpperBounds()
{

}

LineLineParallelDistanceConstraint::LineLineParallelDistanceConstraint(LineShapeDescriptor *fixed, LineShapeDescriptor *constrained, double d):
    LineLineParallelDistanceMinMaxConstraint(fixed, constrained, d, d),
    distance(d)
{
    constraint_name = "line_line_parallel_distance";
    this->line_fixed = fixed;
    this->line_constrained = constrained;
}

LinePointDistanceConstraint::LinePointDistanceConstraint(LineShapeDescriptor *fixed, PointShapeDescriptor *constrained, double d):
    LinePointDistanceMinMaxConstraint(fixed, constrained, d, d),
    distance(d)
{
    constraint_name = "line_point_distance";
    line_fixed = fixed;
    point_constrained = constrained;
}

PlanePlaneAngleConstraint::PlanePlaneAngleConstraint(PlaneShapeDescriptor *fixed, PlaneShapeDescriptor *constrained, double a):
    PlanePlaneAngleMinMaxConstraint(fixed, constrained, a, a),
    angle(a)
{
    constraint_name = "plane_plane_angle";
    plane_fixed = fixed;
    plane_constrained = constrained;
}

PlaneLineAngleConstraint::PlaneLineAngleConstraint(PlaneShapeDescriptor *fixed, LineShapeDescriptor *constrained, double a):
    OperationalPositionConstraint("plane_line_angle"),
    plane_fixed(fixed), line_constrained(constrained), angle(a)
{
}

//NODO
int PlaneLineAngleConstraint::getNumConstraints()
{

}

//NODO
Eigen::VectorXd PlaneLineAngleConstraint::calculateConstraintValue(Eigen::VectorXd &vec)
{

}

//NODO
Eigen::MatrixXd PlaneLineAngleConstraint::calculateConstraintDerivative(Eigen::VectorXd &vec)
{

}

Eigen::VectorXd PlaneLineAngleConstraint::calculateConstraintValueAA(Eigen::VectorXd &vec)
{

}

Eigen::MatrixXd PlaneLineAngleConstraint::calculateConstraintDerivativeAA(Eigen::VectorXd &vec)
{

}

//NODO
Eigen::VectorXd PlaneLineAngleConstraint::getLowerBounds()
{

}

//NODO
Eigen::VectorXd PlaneLineAngleConstraint::getUpperBounds()
{

}


PlaneLinePerpendicularDistanceConstraint::PlaneLinePerpendicularDistanceConstraint(PlaneShapeDescriptor *fixed, LineShapeDescriptor *constrained, double d):
    OperationalPositionConstraint("plane_line_parallel_distance"),
    plane_fixed(fixed), line_constrained(constrained), distance(d)
{
}

//NODO
int PlaneLinePerpendicularDistanceConstraint::getNumConstraints()
{

}

//NODO
Eigen::VectorXd PlaneLinePerpendicularDistanceConstraint::calculateConstraintValue(Eigen::VectorXd &vec)
{

}

//NODO
Eigen::MatrixXd PlaneLinePerpendicularDistanceConstraint::calculateConstraintDerivative(Eigen::VectorXd &vec)
{

}

Eigen::VectorXd PlaneLinePerpendicularDistanceConstraint::calculateConstraintValueAA(Eigen::VectorXd &vec)
{

}

Eigen::MatrixXd PlaneLinePerpendicularDistanceConstraint::calculateConstraintDerivativeAA(Eigen::VectorXd &vec)
{

}

//NODO
Eigen::VectorXd PlaneLinePerpendicularDistanceConstraint::getLowerBounds()
{

}

//NODO
Eigen::VectorXd PlaneLinePerpendicularDistanceConstraint::getUpperBounds()
{

}


PlanePointCoincidentConstraint::PlanePointCoincidentConstraint():
    PlanePointDistanceMinMaxConstraint()
{

}

PlanePointCoincidentConstraint::PlanePointCoincidentConstraint(PlaneShapeDescriptor *fixed, PointShapeDescriptor *constrained):
    PlanePointDistanceMinMaxConstraint(fixed, constrained, 0, 0)
{
    plane_fixed = fixed;
    point_constrained = constrained;
}

PointLineDistanceConstraint::PointLineDistanceConstraint(PointShapeDescriptor *fixed, LineShapeDescriptor *constrained, double d):
    PointLineDistanceMinMaxConstraint(fixed, constrained, d, d),
    distance(d)
{
    this->point_fixed = fixed;
    this->line_constrained = constrained;
}

PlanePlaneCoincidentConstraint::PlanePlaneCoincidentConstraint(PlaneShapeDescriptor *fixed, PlaneShapeDescriptor *constrained):
    PlanePlaneDistanceConstraint(fixed, constrained, 0)
{
    constraint_name = "plane_plane_coincident";
}

CylinderCylinderConcentricConstraint::CylinderCylinderConcentricConstraint(CylinderShapeDescriptor *fixed, CylinderShapeDescriptor *constrained):
        CylinderCylinderParallelDistanceMinMaxConstraint(fixed, constrained, 0, 0)
{
}

CylinderCylinderParallelConstraint::CylinderCylinderParallelConstraint(CylinderShapeDescriptor *fixed, CylinderShapeDescriptor *constrained):
    CylinderCylinderAngleConstraint(fixed, constrained, 0)
{
}


CylinderCylinderAngleConstraint::CylinderCylinderAngleConstraint(CylinderShapeDescriptor *fixed, CylinderShapeDescriptor *constrained, double a):
    CylinderCylinderAngleMinMaxConstraint(fixed, constrained, a, a),
    angle(a)
{
}

//TODO
CylinderCylinderAngleMinMaxConstraint::CylinderCylinderAngleMinMaxConstraint(CylinderShapeDescriptor *fixed, CylinderShapeDescriptor *constrained, double a_min, double a_max):
    OperationalPositionConstraint("cylinder_cylinder_angle_min_max"),
    cylinder_constrained(constrained), cylinder_fixed(fixed), angle_min(a_min), angle_max(a_max)
{

}

//TODO
int CylinderCylinderAngleMinMaxConstraint::getNumConstraints()
{

}

//TODO
Eigen::VectorXd CylinderCylinderAngleMinMaxConstraint::calculateConstraintValue(Eigen::VectorXd &vec)
{

}

//TODO
Eigen::MatrixXd CylinderCylinderAngleMinMaxConstraint::calculateConstraintDerivative(Eigen::VectorXd &vec)
{

}

Eigen::VectorXd CylinderCylinderAngleMinMaxConstraint::calculateConstraintValueAA(Eigen::VectorXd &vec)
{

}

Eigen::MatrixXd CylinderCylinderAngleMinMaxConstraint::calculateConstraintDerivativeAA(Eigen::VectorXd &vec)
{

}

//TODO
Eigen::VectorXd CylinderCylinderAngleMinMaxConstraint::getLowerBounds()
{

}

//TODO
Eigen::VectorXd CylinderCylinderAngleMinMaxConstraint::getUpperBounds()
{

}


LineSegmentPointCoincidentConstraint::LineSegmentPointCoincidentConstraint(LineSegmentShapeDescriptor *fixed, PointShapeDescriptor *constrained):
    OperationalPositionConstraint("linesegment_point_coincident"),
    linesegment_fixed(fixed),
    point_constrained(constrained)
{
}

//TODO
int LineSegmentPointCoincidentConstraint::getNumConstraints()
{

}

//TODO
Eigen::VectorXd LineSegmentPointCoincidentConstraint::calculateConstraintValue(Eigen::VectorXd &vec)
{

}

//TODO
Eigen::MatrixXd LineSegmentPointCoincidentConstraint::calculateConstraintDerivative(Eigen::VectorXd &vec)
{

}

//TODO
Eigen::VectorXd LineSegmentPointCoincidentConstraint::calculateConstraintValueAA(Eigen::VectorXd &vec)
{

}

//TODO
Eigen::MatrixXd LineSegmentPointCoincidentConstraint::calculateConstraintDerivativeAA(Eigen::VectorXd &vec)
{

}

//TODO
Eigen::VectorXd LineSegmentPointCoincidentConstraint::getLowerBounds()
{

}

//TODO
Eigen::VectorXd LineSegmentPointCoincidentConstraint::getUpperBounds()
{

}
