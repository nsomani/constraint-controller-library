

#ifndef OBJECTMODEL_H
#define OBJECTMODEL_H

#include "ConstraintUtils.h"
#include <stdio.h>
#include <iostream>

class PrimitiveShapeDescriptor
{
public:
    Eigen::Matrix4d transformation;
    Eigen::VectorXd model_params;
    std::string model_type;
    PrimitiveShapeDescriptor *ps_parent;
    std::vector<PrimitiveShapeDescriptor*> ps_children;
    int ps_id;
    int dof;
    virtual void transform(Eigen::Matrix4d T);
    virtual void transformChildren(Eigen::Matrix4d T);
};

class PointShapeDescriptor: public PrimitiveShapeDescriptor
{
public:
    Eigen::Vector3d point;
    PointShapeDescriptor(Eigen::Vector3d p);
    PointShapeDescriptor();
    void transform(Eigen::Matrix4d T);
private:
    Eigen::Vector3d point_;
};

class VectorShapeDescriptor: public PrimitiveShapeDescriptor
{
public:
    VectorShapeDescriptor(Eigen::Vector3d v);
    VectorShapeDescriptor();
    void transform(Eigen::Matrix4d T);
    Eigen::Vector3d vec;
private:
    Eigen::Vector3d vec_;
};

class LineShapeDescriptor: public PrimitiveShapeDescriptor
{
public:
    PointShapeDescriptor *point;
    VectorShapeDescriptor *axis;
    LineShapeDescriptor(Eigen::Vector3d p, Eigen::Vector3d a);
    LineShapeDescriptor(PointShapeDescriptor *&p, VectorShapeDescriptor *&a);
    LineShapeDescriptor();
    void transform(Eigen::Matrix4d T);
private:
    PointShapeDescriptor *point_;
    VectorShapeDescriptor *axis_;
};

class LineSegmentShapeDescriptor: public PrimitiveShapeDescriptor
{
public:
    PointShapeDescriptor *point;
    VectorShapeDescriptor *axis;
    double length;
    LineSegmentShapeDescriptor(Eigen::Vector3d p_center, Eigen::Vector3d a, double l);
    LineSegmentShapeDescriptor(PointShapeDescriptor *&p_center, VectorShapeDescriptor *&a, double l);
    LineSegmentShapeDescriptor(Eigen::Vector3d p_start, Eigen::Vector3d p_end);
    LineSegmentShapeDescriptor();
    void transform(Eigen::Matrix4d T);
private:
    PointShapeDescriptor *point_;
    VectorShapeDescriptor *axis_;
};

class PlaneShapeDescriptor: public PrimitiveShapeDescriptor
{
public:
    PointShapeDescriptor *point;
    VectorShapeDescriptor *normal_direction;
    PlaneShapeDescriptor(Eigen::Vector3d p, Eigen::Vector3d n);
    PlaneShapeDescriptor(PointShapeDescriptor *&p, VectorShapeDescriptor *&n);
    PlaneShapeDescriptor();    
    void transform(Eigen::Matrix4d T);
private:
    PointShapeDescriptor *point_;
    VectorShapeDescriptor *normal_direction_;
};

class BoxShapeDescriptor: public PrimitiveShapeDescriptor
{
public:
    PointShapeDescriptor *center;
    Eigen::Vector3d dimensions;
    Eigen::Matrix3d rotation_matrix;
    BoxShapeDescriptor(Eigen::Vector3d c, Eigen::Vector3d dim, Eigen::Matrix3d rot);
    BoxShapeDescriptor(PointShapeDescriptor *&c, Eigen::Vector3d dim, Eigen::Matrix3d rot);
    BoxShapeDescriptor();
    void transform(Eigen::Matrix4d T);
private:
    PointShapeDescriptor *center_;
    Eigen::Matrix3d rotation_matrix_;
};

class CylinderShapeDescriptor: public PrimitiveShapeDescriptor
{
public:
    double radius;
    double height;
    PointShapeDescriptor *center;
    VectorShapeDescriptor *axis;
    CylinderShapeDescriptor();
    CylinderShapeDescriptor(Eigen::Vector3d c, Eigen::Vector3d n, double r, double h);
    CylinderShapeDescriptor(PointShapeDescriptor *&c, VectorShapeDescriptor *&a, double r, double h);
    void transform(Eigen::Matrix4d T);
private:
    PointShapeDescriptor *center_;
    VectorShapeDescriptor *axis_;
};

class ThickCylinderShapeDescriptor: public PrimitiveShapeDescriptor
{
public:
    double radius;
    double height;
    double width;
    PointShapeDescriptor *center;
    VectorShapeDescriptor *axis;
    ThickCylinderShapeDescriptor();
    ThickCylinderShapeDescriptor(Eigen::Vector3d c, Eigen::Vector3d a, double r, double h, double w);
    ThickCylinderShapeDescriptor(PointShapeDescriptor *&c, VectorShapeDescriptor *&a, double r, double h, double w);
    void transform(Eigen::Matrix4d T);
private:
    PointShapeDescriptor *center_;
    VectorShapeDescriptor *axis_;
};

class ConeShapeDescriptor: public PrimitiveShapeDescriptor
{
public:
    double radius;
    double angle;
    PointShapeDescriptor *center;
    VectorShapeDescriptor *axis;
    ConeShapeDescriptor();
    ConeShapeDescriptor(Eigen::Vector3d c, Eigen::Vector3d ax, double r, double a);
    ConeShapeDescriptor(PointShapeDescriptor *&c, VectorShapeDescriptor *&ax, double r, double a);
    void transform(Eigen::Matrix4d T);
private:
    PointShapeDescriptor *center_;
    VectorShapeDescriptor *axis_;

};

class SphereShapeDescriptor: public PrimitiveShapeDescriptor
{
public:
    double radius;
    PointShapeDescriptor *center;
    SphereShapeDescriptor();
    SphereShapeDescriptor(Eigen::Vector3d c, double r);
    SphereShapeDescriptor(PointShapeDescriptor *&c, double r);
    void transform(Eigen::Matrix4d T);
private:
    PointShapeDescriptor *center_;
};

class SphericalShellShapeDescriptor: public PrimitiveShapeDescriptor
{
public:
    double radius_min, radius_max;
    PointShapeDescriptor *center;
    SphericalShellShapeDescriptor();
    SphericalShellShapeDescriptor(Eigen::Vector3d c, double r_min, double r_max);
    SphericalShellShapeDescriptor(PointShapeDescriptor *&c, double r_min, double r_max);
    void transform(Eigen::Matrix4d T);
private:
    PointShapeDescriptor *center_;
};

class CircleShapeDescriptor: public PrimitiveShapeDescriptor
{
public:
    double radius;
    PointShapeDescriptor *center;
    VectorShapeDescriptor *normal;
    CircleShapeDescriptor();
    CircleShapeDescriptor(Eigen::Vector3d n, Eigen::Vector3d c,double r);
    CircleShapeDescriptor(VectorShapeDescriptor *&n, PointShapeDescriptor *&c,double r);
    void transform(Eigen::Matrix4d T);
private:
    PointShapeDescriptor *center_;
    VectorShapeDescriptor *normal_;
};

class EllipseShapeDescriptor: public PrimitiveShapeDescriptor
{
public:
    double radius_major, radius_minor;
    PointShapeDescriptor *center;
    VectorShapeDescriptor *axis_major, *axis_minor, *normal;
    EllipseShapeDescriptor();
    EllipseShapeDescriptor(Eigen::Vector3d n, Eigen::Vector3d a_major, Eigen::Vector3d a_minor, Eigen::Vector3d c, double r_major, double r_minor);
    EllipseShapeDescriptor(VectorShapeDescriptor *&n, VectorShapeDescriptor *&a_major, VectorShapeDescriptor *&a_minor, PointShapeDescriptor *&c, double r_major, double r_minor);
    void transform(Eigen::Matrix4d T);
private:
    PointShapeDescriptor *center_;
    VectorShapeDescriptor *axis_major_, *axis_minor_, *normal_;
};

class ObjectModel
{

public:
    ObjectModel();
    ObjectModel(std::string name);
    std::vector<PrimitiveShapeDescriptor*> primitive_shapes;
    std::string name;
    Eigen::Matrix4d transformation;
    void transform(Eigen::Matrix4d T);
};

#endif // OBJECTMODEL_H
