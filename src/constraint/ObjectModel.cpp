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


#include "ObjectModel.h"

ObjectModel::ObjectModel()
{

}

ObjectModel::ObjectModel(std::string name):
    name(name)
{
    transformation.setIdentity();
}

void ObjectModel::transform(Eigen::Matrix4d T)
{
    for(std::size_t ps_id = 0;ps_id < primitive_shapes.size();++ps_id)
    {
//        primitive_shapes[ps_id]->transform(primitive_shapes[ps_id]->transformation.inverse());
        primitive_shapes[ps_id]->transform(T);
    }
    transformation = T;
}

void PrimitiveShapeDescriptor::transform(Eigen::Matrix4d T)
{
    transformChildren(T);
}

void PrimitiveShapeDescriptor::transformChildren(Eigen::Matrix4d T)
{
    for(std::size_t child_id = 0;child_id < ps_children.size();++child_id)
        ps_children[child_id]->transform(T);
}

PointShapeDescriptor::PointShapeDescriptor()
{
    dof = 0;
    model_type = "point";
}

void PointShapeDescriptor::transform(Eigen::Matrix4d T)
{
    transformation = T;
    point = T.block(0,0,3,3)*point_+T.block(0,3,3,1);
    model_params << 0,0,0, point, 0, 0;
    transformChildren(T);
}

PointShapeDescriptor::PointShapeDescriptor(Eigen::Vector3d p):
    point(p), point_(p)
{
    dof = 0;
    model_type = "point";
    model_params.resize(8);
    model_params << 0,0,0, point, 0, 0;
    transformation.setIdentity();
}


VectorShapeDescriptor::VectorShapeDescriptor(Eigen::Vector3d v):
    vec(v), vec_(v)
{
    dof = 0;
    model_type = "vector";
}

VectorShapeDescriptor::VectorShapeDescriptor()
{
    dof = 0;
    model_type = "vector";
}

void VectorShapeDescriptor::transform(Eigen::Matrix4d T)
{
    vec = T.block(0,0,3,3)*vec_;
}

LineShapeDescriptor::LineShapeDescriptor()
{
    dof = 1;
    model_type = "line";
}

LineShapeDescriptor::LineShapeDescriptor(PointShapeDescriptor *&p, VectorShapeDescriptor *&a):
    point(p), axis(a),
    point_(new PointShapeDescriptor(p->point)), axis_(new VectorShapeDescriptor(a->vec))
{
    dof = 1;
    model_type = "line";
    model_params.resize(7);
    model_params << axis->vec, point->point, 0;
    transformation.setIdentity();
}

LineShapeDescriptor::LineShapeDescriptor(Eigen::Vector3d p, Eigen::Vector3d a):
    point(new PointShapeDescriptor(p)), axis(new VectorShapeDescriptor(a)),
    point_(new PointShapeDescriptor(p)), axis_(new VectorShapeDescriptor(a))
{
    dof = 1;
    model_type = "line";
    model_params.resize(7);
    model_params << axis->vec, point->point, 0;
    transformation.setIdentity();
}

void LineShapeDescriptor::transform(Eigen::Matrix4d T)
{
    transformation = T;
    point->point = T.block(0,0,3,3)*point_->point+T.block(0,3,3,1);
    axis->vec = T.block(0,0,3,3)*axis_->vec;
    model_params << axis->vec, point->point, 0;
    transformChildren(T);
}

LineSegmentShapeDescriptor::LineSegmentShapeDescriptor(Eigen::Vector3d p_center, Eigen::Vector3d a, double l):
    point(new PointShapeDescriptor(p_center)), axis(new VectorShapeDescriptor(a)), length(l),
    point_(new PointShapeDescriptor(p_center)), axis_(new VectorShapeDescriptor(a))
{
    dof = 1;
    model_type = "line_segment";
    model_params.resize(7);
    model_params << axis->vec, point->point, 0;
    transformation.setIdentity();
}

LineSegmentShapeDescriptor::LineSegmentShapeDescriptor(PointShapeDescriptor *&p_center, VectorShapeDescriptor *&a, double l):
    point(p_center), axis(a), length(l),
    point_(new PointShapeDescriptor(p_center->point)), axis_(new VectorShapeDescriptor(a->vec))
{
    dof = 1;
    model_type = "line_segment";
    model_params.resize(7);
    model_params << axis->vec, point->point, 0;
    transformation.setIdentity();
}

LineSegmentShapeDescriptor::LineSegmentShapeDescriptor(Eigen::Vector3d p_start, Eigen::Vector3d p_end):
    point(new PointShapeDescriptor(0.5*(p_start+p_end))), axis(new VectorShapeDescriptor((p_end-p_start).normalized())), length((p_end-p_start).norm()),
    point_(new PointShapeDescriptor(0.5*(p_start+p_end))), axis_(new VectorShapeDescriptor((p_end-p_start).normalized()))
{
    dof = 1;
    model_type = "line_segment";
    model_params.resize(7);
    model_params << axis->vec, point->point, 0;
    transformation.setIdentity();
}

LineSegmentShapeDescriptor::LineSegmentShapeDescriptor()
{
    dof = 1;
    model_type = "line_segment";
}

void LineSegmentShapeDescriptor::transform(Eigen::Matrix4d T)
{
    transformation = T;
    point->point = T.block(0,0,3,3)*point_->point+T.block(0,3,3,1);
    axis->vec = T.block(0,0,3,3)*axis_->vec;
    model_params << axis->vec, point->point, 0;
    transformChildren(T);
}

PlaneShapeDescriptor::PlaneShapeDescriptor()
{
    dof = 2;
    model_type = "plane";
}

void PlaneShapeDescriptor::transform(Eigen::Matrix4d T)
{
    transformation = T;
    point->point = T.block(0,0,3,3)*point_->point+T.block(0,3,3,1);
    normal_direction->vec = T.block(0,0,3,3)*normal_direction_->vec;
    model_params << normal_direction->vec, point->point, 0;
    transformChildren(T);
}

PlaneShapeDescriptor::PlaneShapeDescriptor(Eigen::Vector3d p, Eigen::Vector3d n):
    point(new PointShapeDescriptor(p)), normal_direction(new VectorShapeDescriptor(n)),
    point_(new PointShapeDescriptor(p)), normal_direction_(new VectorShapeDescriptor(n))
{
    dof = 2;
    model_type = "plane";
    model_params.resize(7);
    model_params << normal_direction->vec, point->point, 0;
    transformation.setIdentity();
}

PlaneShapeDescriptor::PlaneShapeDescriptor(PointShapeDescriptor *&p, VectorShapeDescriptor *&n):
    point(p), normal_direction(n),
    point_(new PointShapeDescriptor(p->point)), normal_direction_(new VectorShapeDescriptor(n->vec))
{
    dof = 2;
    model_type = "plane";
    model_params.resize(7);
    model_params << normal_direction->vec, point->point, 0;
    transformation.setIdentity();
}

BoxShapeDescriptor::BoxShapeDescriptor(Eigen::Vector3d c, Eigen::Vector3d dim, Eigen::Matrix3d rot):
    center(new PointShapeDescriptor(c)), dimensions(dim), rotation_matrix(rot),
    center_(new PointShapeDescriptor(c)), rotation_matrix_(rot)
{
    dof = 3;
    model_type = "box";
    model_params.resize(9);
    model_params << rotation_matrix.eulerAngles(0,1,2), center, dimensions;
    transformation.setIdentity();
}

BoxShapeDescriptor::BoxShapeDescriptor(PointShapeDescriptor *&c, Eigen::Vector3d dim, Eigen::Matrix3d rot):
    center(c), dimensions(dim), rotation_matrix(rot),
    center_(new PointShapeDescriptor(c->point)), rotation_matrix_(rot)
{
    dof = 3;
    model_type = "box";
    model_params.resize(9);
}

BoxShapeDescriptor::BoxShapeDescriptor()
{
    dof = 3;
    model_type = "box";
}

void BoxShapeDescriptor::transform(Eigen::Matrix4d T)
{
    transformation = T;
    center->point = T.block(0,0,3,3)*center_->point+T.block(0,3,3,1);
    rotation_matrix = T.block(0,0,3,3)*rotation_matrix_;
//    model_params << rotation_matrix.eulerAngles(0,1,2), center, dimensions;
    transformChildren(T);
}

CylinderShapeDescriptor::CylinderShapeDescriptor()
{
    dof = 2;
    model_type = "cylinder";
}

CylinderShapeDescriptor::CylinderShapeDescriptor(Eigen::Vector3d c, Eigen::Vector3d a, double r, double h):
    center(new PointShapeDescriptor(c)), axis(new VectorShapeDescriptor(a)), radius(r), height(h),
    center_(new PointShapeDescriptor(c)), axis_(new VectorShapeDescriptor(a))
{
    dof = 2;
    model_type = "cylinder";
    model_params.resize(8);
    model_params << axis->vec, center->point, radius, height;
    transformation.setIdentity();
}

CylinderShapeDescriptor::CylinderShapeDescriptor(PointShapeDescriptor *&c, VectorShapeDescriptor *&a, double r, double h):
    center(c), axis(a), radius(r), height(h),
    center_(new PointShapeDescriptor(c->point)), axis_(new VectorShapeDescriptor(a->vec))
{
    dof = 2;
    model_type = "cylinder";
    model_params.resize(8);
    model_params << axis->vec, center->point, radius, height;
    transformation.setIdentity();
}

void CylinderShapeDescriptor::transform(Eigen::Matrix4d T)
{
    transformation = T;
    center->point = T.block(0,0,3,3)*center_->point+T.block(0,3,3,1);
    axis->vec = T.block(0,0,3,3)*axis_->vec;
    model_params << axis->vec, center->point, radius, height;
    transformChildren(T);
}

ThickCylinderShapeDescriptor::ThickCylinderShapeDescriptor()
{
    dof = 3;
    model_type = "thickcylinder";
}

ThickCylinderShapeDescriptor::ThickCylinderShapeDescriptor(Eigen::Vector3d c, Eigen::Vector3d a, double r, double h, double w):
    center(new PointShapeDescriptor(c)), axis(new VectorShapeDescriptor(a)), radius(r), height(h), width(w),
    center_(new PointShapeDescriptor(c)), axis_(new VectorShapeDescriptor(a))
{
    dof = 3;
    model_type = "thickcylinder";
    model_params.resize(9);
    model_params << axis->vec, center->point, radius, height, width;
    transformation.setIdentity();
}

ThickCylinderShapeDescriptor::ThickCylinderShapeDescriptor(PointShapeDescriptor *&c, VectorShapeDescriptor *&a, double r, double h, double w):
    center(c), axis(a), radius(r), height(h), width(w),
    center_(new PointShapeDescriptor(c->point)), axis_(new VectorShapeDescriptor(a->vec))
{
    dof = 3;
    model_type = "thickcylinder";
    model_params.resize(9);
    model_params << axis->vec, center->point, radius, height, width;
    transformation.setIdentity();
}

void ThickCylinderShapeDescriptor::transform(Eigen::Matrix4d T)
{
    transformation = T;
    center->point = T.block(0,0,3,3)*center_->point+T.block(0,3,3,1);
    axis->vec = T.block(0,0,3,3)*axis_->vec;
    model_params << axis->vec, center->point, radius, height, width;
    transformChildren(T);
}

ConeShapeDescriptor::ConeShapeDescriptor()
{
    dof = 2;
    model_type = "cone";
}

ConeShapeDescriptor::ConeShapeDescriptor(Eigen::Vector3d c, Eigen::Vector3d ax, double r, double a):
    center(new PointShapeDescriptor(c)), axis(new VectorShapeDescriptor(ax)), radius(r), angle(a),
    center_(new PointShapeDescriptor(c)), axis_(new VectorShapeDescriptor(ax))
{
    dof = 2;
    model_type = "cone";
    model_params.resize(8);
    model_params << axis->vec, center->point, radius, angle;
    transformation.setIdentity();
}

ConeShapeDescriptor::ConeShapeDescriptor(PointShapeDescriptor *&c, VectorShapeDescriptor *&ax, double r, double a):
    center(c), axis(ax), radius(r), angle(a),
    center_(new PointShapeDescriptor(c->point)), axis_(new VectorShapeDescriptor(ax->vec))
{
    dof = 2;
    model_type = "cone";
    model_params.resize(8);
    model_params << axis->vec, center->point, radius, angle;
    transformation.setIdentity();
}

void ConeShapeDescriptor::transform(Eigen::Matrix4d T)
{
    transformation = T;
    center->point = T.block(0,0,3,3)*center_->point+T.block(0,3,3,1);
    axis->vec = T.block(0,0,3,3)*axis_->vec;
    model_params << axis->vec, center->point, radius, angle;
    transformChildren(T);
}


SphereShapeDescriptor::SphereShapeDescriptor()
{
    dof = 2;
    model_type = "sphere";
}

SphereShapeDescriptor::SphereShapeDescriptor(Eigen::Vector3d c, double r):
    center(new PointShapeDescriptor(c)), radius(r),
    center_(new PointShapeDescriptor(c))
{
    dof = 2;
    model_type = "sphere";
    model_params.resize(4);
    model_params << center->point, radius;
    transformation.setIdentity();
}

SphereShapeDescriptor::SphereShapeDescriptor(PointShapeDescriptor *&c, double r):
    center(c), radius(r),
    center_(new PointShapeDescriptor(c->point))
{
    dof = 2;
    model_type = "sphere";
    model_params.resize(4);
    model_params << center->point, radius;
    transformation.setIdentity();
}

void SphereShapeDescriptor::transform(Eigen::Matrix4d T)
{
    transformation = T;
    center->point = T.block(0,0,3,3)*center_->point+T.block(0,3,3,1);
    model_params << center->point, radius;
    transformChildren(T);
}

SphericalShellShapeDescriptor::SphericalShellShapeDescriptor()
{
    dof = 3;
    model_type = "sphericalshell";
}

SphericalShellShapeDescriptor::SphericalShellShapeDescriptor(Eigen::Vector3d c, double r_min, double r_max):
    center(new PointShapeDescriptor(c)), radius_min(r_min), radius_max(r_max),
    center_(new PointShapeDescriptor(c))
{
    dof = 3;
    model_type = "sphericalshell";
    model_params.resize(5);
    model_params << center->point, radius_min, radius_max;
    transformation.setIdentity();
}

SphericalShellShapeDescriptor::SphericalShellShapeDescriptor(PointShapeDescriptor *&c, double r_min, double r_max):
    center(c), radius_min(r_min), radius_max(r_max),
    center_(new PointShapeDescriptor(c->point))
{
    dof = 3;
    model_type = "sphericalshell";
    model_params.resize(5);
    model_params << center->point, radius_min, radius_max;
    transformation.setIdentity();
}

void SphericalShellShapeDescriptor::transform(Eigen::Matrix4d T)
{
    transformation = T;
    center->point = T.block(0,0,3,3)*center_->point+T.block(0,3,3,1);
    model_params << center->point, radius_min, radius_max;
    transformChildren(T);
}

CircleShapeDescriptor::CircleShapeDescriptor()
{
    dof = 1;
    model_type = "circle";
}

CircleShapeDescriptor::CircleShapeDescriptor(Eigen::Vector3d n, Eigen::Vector3d c, double r):
    center(new PointShapeDescriptor(c)), normal(new VectorShapeDescriptor(n)), radius(r),
    center_(new PointShapeDescriptor(c)), normal_(new VectorShapeDescriptor(n))
{
    dof = 1;
    model_type = "circle";
    model_params.resize(7);
    model_params << center->point, normal, radius;
    transformation.setIdentity();
}

CircleShapeDescriptor::CircleShapeDescriptor(VectorShapeDescriptor *&n, PointShapeDescriptor *&c, double r):
    center(c), normal(n), radius(r),
    center_(new PointShapeDescriptor(c->point)), normal_(new VectorShapeDescriptor(n->vec))
{
    dof = 1;
    model_type = "circle";
    model_params.resize(7);
    model_params << center->point, normal->vec, radius;
    transformation.setIdentity();
}

void CircleShapeDescriptor::transform(Eigen::Matrix4d T)
{
    transformation = T;
    center->point = T.block(0,0,3,3)*center_->point+T.block(0,3,3,1);
    normal->vec = T.block(0,0,3,3)*normal_->vec;
    model_params << center->point, normal->vec, radius;
    transformChildren(T);
}

EllipseShapeDescriptor::EllipseShapeDescriptor()
{
    dof = 1;
    model_type = "ellipse";
}

EllipseShapeDescriptor::EllipseShapeDescriptor(Eigen::Vector3d n, Eigen::Vector3d a_major, Eigen::Vector3d a_minor, Eigen::Vector3d c, double r_major, double r_minor):
    center(new PointShapeDescriptor(c)), normal(new VectorShapeDescriptor(n)), axis_major(new VectorShapeDescriptor(a_major)), axis_minor(new VectorShapeDescriptor(a_minor)), radius_major(r_major), radius_minor(r_minor),
    center_(new PointShapeDescriptor(c)), normal_(new VectorShapeDescriptor(n)), axis_major_(new VectorShapeDescriptor(a_major)), axis_minor_(new VectorShapeDescriptor(a_minor))
{
    dof = 1;
    model_type = "ellipse";
    transformation.setIdentity();
    model_params.resize(14);
    model_params << center->point, normal, axis_major, axis_minor, radius_major, radius_minor;
}

EllipseShapeDescriptor::EllipseShapeDescriptor(VectorShapeDescriptor *&n, VectorShapeDescriptor *&a_major, VectorShapeDescriptor *&a_minor, PointShapeDescriptor *&c, double r_major, double r_minor):
    center(c), normal(n), axis_major(a_major), axis_minor(a_minor), radius_major(r_major), radius_minor(r_minor),
    center_(new PointShapeDescriptor(c->point)), normal_(new VectorShapeDescriptor(n->vec)), axis_major_(new VectorShapeDescriptor(a_major->vec)), axis_minor_(new VectorShapeDescriptor(a_minor->vec))
{
    dof = 1;
    model_type = "ellipse";
    transformation.setIdentity();
    model_params.resize(14);
    model_params << center->point, normal, axis_major, axis_minor, radius_major, radius_minor;
}

void EllipseShapeDescriptor::transform(Eigen::Matrix4d T)
{
    transformation = T;
    center->point = T.block(0,0,3,3)*center_->point+T.block(0,3,3,1);
    normal->vec = T.block(0,0,3,3)*normal_->vec;
    axis_major->vec = T.block(0,0,3,3)*axis_major_->vec;
    axis_minor->vec = T.block(0,0,3,3)*axis_minor_->vec;
    model_params << center->point, normal->vec, axis_major->vec, axis_minor->vec, radius_major, radius_minor;
    transformChildren(T);
}
