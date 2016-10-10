

#ifndef CONSTRAINTUTILS_H
#define CONSTRAINTUTILS_H

//#include <Eigen/Dense>
#include <rl/math/Transform.h>
#include <rl/math/Rotation.h>
#include <rl/mdl/Dynamic.h>
#include <rl/mdl/Model.h>
#include <rl/mdl/Body.h>
#define FLOAT_EPS 1e-6

#include <iostream>

inline bool floatEqual(double a, double b) {
    return std::fabs(a - b) < FLOAT_EPS;
}

inline bool floatLessThan(double a, double b) {
    return ((std::fabs(a - b) > FLOAT_EPS) && ((a - b) < FLOAT_EPS));
}

inline bool floatGreaterThan(double a, double b) {
    return ((std::fabs(a - b) > FLOAT_EPS) && ((a - b) > FLOAT_EPS));
}

inline Eigen::Vector3d getOrthonormal(Eigen::Vector3d x)
{
    // Numerically stable orthonormal
    Eigen::Vector3d some_unit(1,0,0);
    Eigen::Vector3d some_other_unit(0,1,0);
    if((x.transpose() * some_unit).norm() < (x.transpose() * some_other_unit).norm())
    {
        return x.cross(some_unit).normalized();
    }
    else
    {
        return x.cross(some_other_unit).normalized();
    }
}

inline void getOrthonormals(Eigen::Vector3d x, Eigen::Vector3d &x_n1, Eigen::Vector3d &x_n2)
{
    // Numerically stable orthonormals
    Eigen::Vector3d y = getOrthonormal(x);
    x_n1 = y;
    x_n2 = x.cross(y).normalized();
}

inline double safeAcos (double x)
{
    if (x < -1.0) x = -1.0 ;
    else if (x > 1.0) x = 1.0 ;
    return std::acos (x) ;
}

inline Eigen::Matrix3d getdRx(Eigen::Vector4d axisAngle)
{
    double wx = axisAngle(0);
    double wy = axisAngle(1);
    double wz = axisAngle(2);
    double theta = axisAngle(3);
    Eigen::Matrix3d dRx;
    dRx << -2*wx*(std::cos(theta) - 1), -wy*(std::cos(theta) - 1), -wz*(std::cos(theta) - 1),
           -wy*(std::cos(theta) - 1)  ,                         0,         -std::sin(theta),
           -wz*(std::cos(theta) - 1)  ,           std::sin(theta),                         0;

    return dRx;
}

inline Eigen::Matrix3d getdRy(Eigen::Vector4d axisAngle)
{
    double wx = axisAngle(0);
    double wy = axisAngle(1);
    double wz = axisAngle(2);
    double theta = axisAngle(3);
    Eigen::Matrix3d dRy;
    dRy << 0                       , -wx*(std::cos(theta) - 1)  ,           std::sin(theta),
          -wx*(std::cos(theta) - 1), -2*wy*(std::cos(theta) - 1), -wz*(std::cos(theta) - 1),
          -std::sin(theta)         , -wz*(std::cos(theta) - 1)  ,                         0;
    return dRy;
}

inline Eigen::Matrix3d getdRz(Eigen::Vector4d axisAngle)
{
    double wx = axisAngle(0);
    double wy = axisAngle(1);
    double wz = axisAngle(2);
    double theta = axisAngle(3);
    Eigen::Matrix3d dRz;

    dRz <<  0                        , -std::sin(theta)         , -wx*(std::cos(theta) - 1) ,
            std::sin(theta)          , 0                        , -wy*(std::cos(theta) - 1) ,
            -wx*(std::cos(theta) - 1), -wy*(std::cos(theta) - 1), -2*wz*(std::cos(theta) - 1);

    return dRz;
}

inline Eigen::Matrix3d getdRtheta(Eigen::Vector4d axisAngle)
{
    double wx = axisAngle(0);
    double wy = axisAngle(1);
    double wz = axisAngle(2);
    double theta = axisAngle(3);
    Eigen::Matrix3d dRtheta;

    dRtheta << std::sin(theta)*wx*wx - std::sin(theta)   , wx*wy*std::sin(theta) - wz*std::cos(theta), wy*std::cos(theta) + wx*wz*std::sin(theta),
              wz*std::cos(theta) + wx*wy*std::sin(theta), std::sin(theta)*wy*wy - std::sin(theta)   , wy*wz*std::sin(theta) - wx*std::cos(theta),
              wx*wz*std::sin(theta) - wy*std::cos(theta), wx*std::cos(theta) + wy*wz*std::sin(theta), std::sin(theta)*wz*wz - std::sin(theta)   ;
    return dRtheta;
}

inline Eigen::VectorXd vectorFromTransform(Eigen::Matrix4d T)
{
    Eigen::VectorXd X;
    X.resize(7);
    X.segment(0,3) = T.block(0,3,3,1);
    Eigen::Matrix3d R = T.block(0,0,3,3);
    Eigen::AngleAxisd aa(R);
    X.segment(3,3) = aa.axis();
    X(6) = aa.angle();
    while(X(6) > M_PI)
        X(6) -= M_PI;
    while(X(6) < -M_PI)
        X(6) += M_PI;
    return X;
}

inline Eigen::VectorXd quaternionFromTransform(Eigen::Matrix4d T)
{
    Eigen::VectorXd X;
    X.resize(7);
    X.segment(0,3) = T.block(0,3,3,1);
    Eigen::Matrix3d R = T.block(0,0,3,3);
    Eigen::Quaterniond qt(R);
    X(3) = qt.x();
    X(4) = qt.y();
    X(5) = qt.z();
    X(6) = qt.w();
    return X;
}

inline Eigen::Matrix4d transformFromVector(Eigen::VectorXd X)
{
    Eigen::Matrix4d T;
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    R = Eigen::AngleAxisd(X(6), X.segment(3,3));
    t = X.segment(0,3);
    T.setIdentity();
    T.block(0,0,3,3) = R;
    T.block(0,3,3,1) = t;
    return T;
}

inline Eigen::Matrix4d transformFromQuaternion(Eigen::VectorXd X)
{
    Eigen::Matrix4d T;
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    R = Eigen::Quaterniond(X(6), X(3), X(4), X(5)).toRotationMatrix();
    t = X.segment(0,3);
    T.setIdentity();
    T.block(0,0,3,3) = R;
    T.block(0,3,3,1) = t;
    return T;
}

inline Eigen::Matrix3d rotationMatrixFromAA(Eigen::Vector3d axis, double angle)
{
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(angle, axis);
    return R;
}

inline Eigen::Matrix3d rotationMatrixFromAA(Eigen::Vector4d axis_angle)
{
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(axis_angle(3), axis_angle.segment(0,3));
    return R;
}

inline Eigen::Matrix3d rotationMatrixFromBasisVectors(Eigen::Vector3d u, Eigen::Vector3d v, Eigen::Vector3d w)
{
    Eigen::Matrix3d R;
    R.col(0) = u.transpose();
    R.col(1) = v.transpose();
    R.col(2) = w.transpose();
    return R;
}

inline Eigen::Matrix3d rotationMatrixFromVectorPairs(Eigen::Vector3d u_1, Eigen::Vector3d v_1, Eigen::Vector3d u_2, Eigen::Vector3d v_2)
{
    Eigen::Matrix3d R;
    Eigen::Vector3d v_u_1, v_u_2, w_1, w_2;
    v_u_1 = (v_1-(u_1.dot(v_1)*u_1));
    if(floatEqual(std::fabs(u_1.dot(v_1)),1))
    {
        getOrthonormals(u_1, v_u_1, w_1);
    }
    else
    {
        v_u_1 /= v_u_1.norm();
        w_1 = u_1.cross(v_u_1);
        if(floatEqual(w_1.norm(),0))
            w_1 = getOrthonormal(u_1);
        else
            w_1.normalize();
    }
    v_u_2 = (v_2-(u_2.dot(v_2)*u_2));
    if(floatEqual(std::fabs(u_2.dot(v_2)),1))
    {
        getOrthonormals(u_2, v_u_2, w_2);
    }
    else
    {
        v_u_2 /= v_u_2.norm();
        w_2 = u_2.cross(v_u_2);
        if(floatEqual(w_2.norm(),0))
            w_2 = getOrthonormal(u_2);
        else
            w_2.normalize();
    }
    Eigen::Matrix3d R1 = rotationMatrixFromBasisVectors(u_1, v_u_1, w_1);
    Eigen::Matrix3d R2 = rotationMatrixFromBasisVectors(u_2, v_u_2, w_2);
    R = R2*(R1.transpose());
    return R;
}

inline Eigen::Vector3d projectPointToPlane(Eigen::Vector3d point, Eigen::Vector3d plane_point, Eigen::Vector3d plane_normal)
{
    Eigen::Vector3d point_projection = point-plane_point;
    point_projection -= (point_projection.dot(plane_normal))*plane_normal;
    point_projection += plane_point;
    return point_projection;
}

inline Eigen::Vector3d linePlaneIntersection(Eigen::Vector3d line_point, Eigen::Vector3d line_normal, Eigen::Vector3d plane_point, Eigen::Vector3d plane_normal)
{
    double t = (plane_normal.dot(plane_point-line_point))/(line_normal.dot(plane_normal));
    Eigen::Vector3d point_intersection = line_point + t*line_normal;
//    std::cout << "point_intersection:" << point_intersection.transpose() << std::endl;
    return point_intersection;
}

inline Eigen::VectorXd planePlaneIntersection(Eigen::Vector3d plane1_point, Eigen::Vector3d plane1_normal, Eigen::Vector3d plane2_point, Eigen::Vector3d plane2_normal)
{
    Eigen::VectorXd line_intersection;
    line_intersection.resize(6);
    Eigen::Vector3d line_point, line_normal;
    line_normal = plane1_normal.cross(plane2_normal);
    float det = line_normal.squaredNorm();

    if (det != 0.0) {
        line_point = ((line_normal.cross(plane2_normal)*(-1*plane1_point.dot(plane1_normal))) + (plane1_normal.cross(line_normal))*(-1*plane2_point.dot(plane2_normal)))/det;
    }
    line_intersection << line_point, line_normal.normalized();
//    std::cout << "line_intersection:" << line_intersection.transpose() << std::endl;
    return line_intersection;
}

inline Eigen::Vector3d lineLineIntersection(Eigen::Vector3d line1_point, Eigen::Vector3d line1_normal, Eigen::Vector3d line2_point, Eigen::Vector3d line2_normal)
{
    Eigen::Vector3d point_intersection = line1_point + (line1_normal)*(((line1_point-line2_point).cross(line2_normal)).dot(line1_normal.cross(line2_normal)))/(line1_normal.cross(line2_normal).dot(line1_normal.cross(line2_normal)));
    return point_intersection;
}

inline double distanceRotations(Eigen::Vector4d aa1, Eigen::Vector4d aa2)
{
//    double theta1, wx1, wy1, wz1, wx12, wy12, wz12;
//    double theta2, wx2, wy2, wz2, wx22, wy22, wz22;
//    wx1 = aa1(0);wy1 = aa1(1);wz1 = aa1(2);theta1 = aa1(3);
//    wx12 = wx1*wx1;wy12 = wy1*wy1;wz12 = wz1*wz1;
//    wx2 = aa2(0);wy2 = aa2(1);wz2 = aa2(2);theta2 = aa2(3);
//    wx22 = wx2*wx2;wy22 = wy2*wy2;wz22 = wz2*wz2;
//    double theta = safeAcos(((wz1*std::sin(theta1) + wx1*wy1*(std::cos(theta1) - 1))*(wz2*std::sin(theta2) + wx2*wy2*(std::cos(theta2) - 1)))/2 + ((wx1*std::sin(theta1) + wy1*wz1*(std::cos(theta1) - 1))*(wx2*std::sin(theta2) + wy2*wz2*(std::cos(theta2) - 1)))/2 + ((wy1*std::sin(theta1) + wx1*wz1*(std::cos(theta1) - 1))*(wy2*std::sin(theta2) + wx2*wz2*(std::cos(theta2) - 1)))/2 + ((wx1*std::sin(theta1) - wy1*wz1*(std::cos(theta1) - 1))*(wx2*std::sin(theta2) - wy2*wz2*(std::cos(theta2) - 1)))/2 + ((wy1*std::sin(theta1) - wx1*wz1*(std::cos(theta1) - 1))*(wy2*std::sin(theta2) - wx2*wz2*(std::cos(theta2) - 1)))/2 + ((wz1*std::sin(theta1) - wx1*wy1*(std::cos(theta1) - 1))*(wz2*std::sin(theta2) - wx2*wy2*(std::cos(theta2) - 1)))/2 + (((1 - std::cos(theta1))*wx12 + std::cos(theta1))*((1 - std::cos(theta2))*wx22 + std::cos(theta2)))/2 + (((1 - std::cos(theta1))*wy12 + std::cos(theta1))*((1 - std::cos(theta2))*wy22 + std::cos(theta2)))/2 + (((1 - std::cos(theta1))*wz12 + std::cos(theta1))*((1 - std::cos(theta2))*wz22 + std::cos(theta2)))/2 - 1/2);
//    return theta;

//    Eigen::Matrix3d R1,R2,R;
//    R1 = Eigen::AngleAxisd(theta1, aa1.segment(0,3));
//    R2 = Eigen::AngleAxisd(theta2, aa2.segment(0,3));
//    R = R1*(R2.transpose());
//    double theta = R.eulerAngles(0,1,2).transpose()*R.eulerAngles(0,1,2);
//    return theta;

    Eigen::Vector4d q1, q2;
    q1 << aa1(0)*std::sin(aa1(3)/2), aa1(1)*std::sin(aa1(3)/2), aa1(2)*std::sin(aa1(3)/2), std::cos(aa1(3)/2);
    q2 << aa2(0)*std::sin(aa2(3)/2), aa2(1)*std::sin(aa2(3)/2), aa2(2)*std::sin(aa2(3)/2), std::cos(aa2(3)/2);
    return (q1-q2).norm();

}

inline Eigen::Vector4d distanceRotationsDerivative(Eigen::Vector4d aa1, Eigen::Vector4d aa2)
{
    double theta1, wx1, wy1, wz1, wx12, wy12, wz12;
    double theta2, wx2, wy2, wz2, wx22, wy22, wz22;
    wx1 = aa1(0);wy1 = aa1(1);wz1 = aa1(2);theta1 = aa1(3);
    wx12 = wx1*wx1;wy12 = wy1*wy1;wz12 = wz1*wz1;
    wx2 = aa2(0);wy2 = aa2(1);wz2 = aa2(2);theta2 = aa2(3);
    wx22 = wx2*wx2;wy22 = wy2*wy2;wz22 = wz2*wz2;
    Eigen::Matrix3d R1,R2,R;
    R1 = Eigen::AngleAxisd(theta1, aa1.segment(0,3));
    R2 = Eigen::AngleAxisd(theta2, aa2.segment(0,3));
    R = R1*(R2.transpose());

    Eigen::Vector4d theta_d;
    theta_d(0) = -((std::sin(theta1)*(wx2*std::sin(theta2) + wy2*wz2*(std::cos(theta2) - 1)))/2 + (std::sin(theta1)*(wx2*std::sin(theta2) - wy2*wz2*(std::cos(theta2) - 1)))/2 + (wy1*(std::cos(theta1) - 1)*(wz2*std::sin(theta2) + wx2*wy2*(std::cos(theta2) - 1)))/2 - (wy1*(std::cos(theta1) - 1)*(wz2*std::sin(theta2) - wx2*wy2*(std::cos(theta2) - 1)))/2 + (wz1*(std::cos(theta1) - 1)*(wy2*std::sin(theta2) + wx2*wz2*(std::cos(theta2) - 1)))/2 - (wz1*(std::cos(theta1) - 1)*(wy2*std::sin(theta2) - wx2*wz2*(std::cos(theta2) - 1)))/2 - wx1*(std::cos(theta1) - 1)*((1 - std::cos(theta2))*wx22 + std::cos(theta2)))/std::sqrt((1.0 - (((wz1*std::sin(theta1) + wx1*wy1*(std::cos(theta1) - 1))*(wz2*std::sin(theta2) + wx2*wy2*(std::cos(theta2) - 1)))/2 + ((wx1*std::sin(theta1) + wy1*wz1*(std::cos(theta1) - 1))*(wx2*std::sin(theta2) + wy2*wz2*(std::cos(theta2) - 1)))/2 + ((wy1*std::sin(theta1) + wx1*wz1*(std::cos(theta1) - 1))*(wy2*std::sin(theta2) + wx2*wz2*(std::cos(theta2) - 1)))/2 + ((wx1*std::sin(theta1) - wy1*wz1*(std::cos(theta1) - 1))*(wx2*std::sin(theta2) - wy2*wz2*(std::cos(theta2) - 1)))/2 + ((wy1*std::sin(theta1) - wx1*wz1*(std::cos(theta1) - 1))*(wy2*std::sin(theta2) - wx2*wz2*(std::cos(theta2) - 1)))/2 + ((wz1*std::sin(theta1) - wx1*wy1*(std::cos(theta1) - 1))*(wz2*std::sin(theta2) - wx2*wy2*(std::cos(theta2) - 1)))/2 + ((std::cos(theta1) - wx12*(std::cos(theta1) - 1))*(std::cos(theta2) - wx22*(std::cos(theta2) - 1)))/2 + ((std::cos(theta1) - wy12*(std::cos(theta1) - 1))*(std::cos(theta2) - wy22*(std::cos(theta2) - 1)))/2 + ((std::cos(theta1) - wz12*(std::cos(theta1) - 1))*(std::cos(theta2) - wz22*(std::cos(theta2) - 1)))/2 - 1/2)*(((wz1*std::sin(theta1) + wx1*wy1*(std::cos(theta1) - 1))*(wz2*std::sin(theta2) + wx2*wy2*(std::cos(theta2) - 1)))/2 + ((wx1*std::sin(theta1) + wy1*wz1*(std::cos(theta1) - 1))*(wx2*std::sin(theta2) + wy2*wz2*(std::cos(theta2) - 1)))/2 + ((wy1*std::sin(theta1) + wx1*wz1*(std::cos(theta1) - 1))*(wy2*std::sin(theta2) + wx2*wz2*(std::cos(theta2) - 1)))/2 + ((wx1*std::sin(theta1) - wy1*wz1*(std::cos(theta1) - 1))*(wx2*std::sin(theta2) - wy2*wz2*(std::cos(theta2) - 1)))/2 + ((wy1*std::sin(theta1) - wx1*wz1*(std::cos(theta1) - 1))*(wy2*std::sin(theta2) - wx2*wz2*(std::cos(theta2) - 1)))/2 + ((wz1*std::sin(theta1) - wx1*wy1*(std::cos(theta1) - 1))*(wz2*std::sin(theta2) - wx2*wy2*(std::cos(theta2) - 1)))/2 + ((std::cos(theta1) - wx12*(std::cos(theta1) - 1))*(std::cos(theta2) - wx22*(std::cos(theta2) - 1)))/2 + ((std::cos(theta1) - wy12*(std::cos(theta1) - 1))*(std::cos(theta2) - wy22*(std::cos(theta2) - 1)))/2 + ((std::cos(theta1) - wz12*(std::cos(theta1) - 1))*(std::cos(theta2) - wz22*(std::cos(theta2) - 1)))/2 - 1/2)));
    theta_d(1) = -((std::sin(theta1)*(wy2*std::sin(theta2) + wx2*wz2*(std::cos(theta2) - 1)))/2 + (std::sin(theta1)*(wy2*std::sin(theta2) - wx2*wz2*(std::cos(theta2) - 1)))/2 + (wx1*(std::cos(theta1) - 1)*(wz2*std::sin(theta2) + wx2*wy2*(std::cos(theta2) - 1)))/2 - (wx1*(std::cos(theta1) - 1)*(wz2*std::sin(theta2) - wx2*wy2*(std::cos(theta2) - 1)))/2 + (wz1*(std::cos(theta1) - 1)*(wx2*std::sin(theta2) + wy2*wz2*(std::cos(theta2) - 1)))/2 - (wz1*(std::cos(theta1) - 1)*(wx2*std::sin(theta2) - wy2*wz2*(std::cos(theta2) - 1)))/2 - wy1*(std::cos(theta1) - 1)*((1 - std::cos(theta2))*wy22 + std::cos(theta2)))/std::sqrt((1.0 - (((wz1*std::sin(theta1) + wx1*wy1*(std::cos(theta1) - 1))*(wz2*std::sin(theta2) + wx2*wy2*(std::cos(theta2) - 1)))/2 + ((wx1*std::sin(theta1) + wy1*wz1*(std::cos(theta1) - 1))*(wx2*std::sin(theta2) + wy2*wz2*(std::cos(theta2) - 1)))/2 + ((wy1*std::sin(theta1) + wx1*wz1*(std::cos(theta1) - 1))*(wy2*std::sin(theta2) + wx2*wz2*(std::cos(theta2) - 1)))/2 + ((wx1*std::sin(theta1) - wy1*wz1*(std::cos(theta1) - 1))*(wx2*std::sin(theta2) - wy2*wz2*(std::cos(theta2) - 1)))/2 + ((wy1*std::sin(theta1) - wx1*wz1*(std::cos(theta1) - 1))*(wy2*std::sin(theta2) - wx2*wz2*(std::cos(theta2) - 1)))/2 + ((wz1*std::sin(theta1) - wx1*wy1*(std::cos(theta1) - 1))*(wz2*std::sin(theta2) - wx2*wy2*(std::cos(theta2) - 1)))/2 + ((std::cos(theta1) - wx12*(std::cos(theta1) - 1))*(std::cos(theta2) - wx22*(std::cos(theta2) - 1)))/2 + ((std::cos(theta1) - wy12*(std::cos(theta1) - 1))*(std::cos(theta2) - wy22*(std::cos(theta2) - 1)))/2 + ((std::cos(theta1) - wz12*(std::cos(theta1) - 1))*(std::cos(theta2) - wz22*(std::cos(theta2) - 1)))/2 - 1/2)*(((wz1*std::sin(theta1) + wx1*wy1*(std::cos(theta1) - 1))*(wz2*std::sin(theta2) + wx2*wy2*(std::cos(theta2) - 1)))/2 + ((wx1*std::sin(theta1) + wy1*wz1*(std::cos(theta1) - 1))*(wx2*std::sin(theta2) + wy2*wz2*(std::cos(theta2) - 1)))/2 + ((wy1*std::sin(theta1) + wx1*wz1*(std::cos(theta1) - 1))*(wy2*std::sin(theta2) + wx2*wz2*(std::cos(theta2) - 1)))/2 + ((wx1*std::sin(theta1) - wy1*wz1*(std::cos(theta1) - 1))*(wx2*std::sin(theta2) - wy2*wz2*(std::cos(theta2) - 1)))/2 + ((wy1*std::sin(theta1) - wx1*wz1*(std::cos(theta1) - 1))*(wy2*std::sin(theta2) - wx2*wz2*(std::cos(theta2) - 1)))/2 + ((wz1*std::sin(theta1) - wx1*wy1*(std::cos(theta1) - 1))*(wz2*std::sin(theta2) - wx2*wy2*(std::cos(theta2) - 1)))/2 + ((std::cos(theta1) - wx12*(std::cos(theta1) - 1))*(std::cos(theta2) - wx22*(std::cos(theta2) - 1)))/2 + ((std::cos(theta1) - wy12*(std::cos(theta1) - 1))*(std::cos(theta2) - wy22*(std::cos(theta2) - 1)))/2 + ((std::cos(theta1) - wz12*(std::cos(theta1) - 1))*(std::cos(theta2) - wz22*(std::cos(theta2) - 1)))/2 - 1/2)));
    theta_d(2) = -((std::sin(theta1)*(wz2*std::sin(theta2) - wx2*wy2*(std::cos(theta2) - 1)))/2 + (wx1*(std::cos(theta1) - 1)*(wy2*std::sin(theta2) + wx2*wz2*(std::cos(theta2) - 1)))/2 - (wx1*(std::cos(theta1) - 1)*(wy2*std::sin(theta2) - wx2*wz2*(std::cos(theta2) - 1)))/2 + (wy1*(std::cos(theta1) - 1)*(wx2*std::sin(theta2) + wy2*wz2*(std::cos(theta2) - 1)))/2 - (wy1*(std::cos(theta1) - 1)*(wx2*std::sin(theta2) - wy2*wz2*(std::cos(theta2) - 1)))/2 - wz1*(std::cos(theta1) - 1)*((1 - std::cos(theta2))*wz22 + std::cos(theta2)))/std::sqrt((1 - (((wz1*std::sin(theta1) + wx1*wy1*(std::cos(theta1) - 1))*(wz2*std::sin(theta2) + wx2*wy2*(std::cos(theta2) - 1)))/2 + ((wx1*std::sin(theta1) + wy1*wz1*(std::cos(theta1) - 1))*(wx2*std::sin(theta2) + wy2*wz2*(std::cos(theta2) - 1)))/2 + ((wy1*std::sin(theta1) + wx1*wz1*(std::cos(theta1) - 1))*(wy2*std::sin(theta2) + wx2*wz2*(std::cos(theta2) - 1)))/2 + ((wx1*std::sin(theta1) - wy1*wz1*(std::cos(theta1) - 1))*(wx2*std::sin(theta2) - wy2*wz2*(std::cos(theta2) - 1)))/2 + ((wy1*std::sin(theta1) - wx1*wz1*(std::cos(theta1) - 1))*(wy2*std::sin(theta2) - wx2*wz2*(std::cos(theta2) - 1)))/2 + ((wz1*std::sin(theta1) - wx1*wy1*(std::cos(theta1) - 1))*(wz2*std::sin(theta2) - wx2*wy2*(std::cos(theta2) - 1)))/2 + ((std::cos(theta1) - wx12*(std::cos(theta1) - 1))*(std::cos(theta2) - wx22*(std::cos(theta2) - 1)))/2 + ((std::cos(theta1) - wy12*(std::cos(theta1) - 1))*(std::cos(theta2) - wy22*(std::cos(theta2) - 1)))/2 + ((std::cos(theta1) - wz12*(std::cos(theta1) - 1))*(std::cos(theta2) - wz22*(std::cos(theta2) - 1)))/2 - 1/2)*(((wz1*std::sin(theta1) + wx1*wy1*(std::cos(theta1) - 1))*(wz2*std::sin(theta2) + wx2*wy2*(std::cos(theta2) - 1)))/2 + ((wx1*std::sin(theta1) + wy1*wz1*(std::cos(theta1) - 1))*(wx2*std::sin(theta2) + wy2*wz2*(std::cos(theta2) - 1)))/2 + ((wy1*std::sin(theta1) + wx1*wz1*(std::cos(theta1) - 1))*(wy2*std::sin(theta2) + wx2*wz2*(std::cos(theta2) - 1)))/2 + ((wx1*std::sin(theta1) - wy1*wz1*(std::cos(theta1) - 1))*(wx2*std::sin(theta2) - wy2*wz2*(std::cos(theta2) - 1)))/2 + ((wy1*std::sin(theta1) - wx1*wz1*(std::cos(theta1) - 1))*(wy2*std::sin(theta2) - wx2*wz2*(std::cos(theta2) - 1)))/2 + ((wz1*std::sin(theta1) - wx1*wy1*(std::cos(theta1) - 1))*(wz2*std::sin(theta2) - wx2*wy2*(std::cos(theta2) - 1)))/2 + ((std::cos(theta1) - wx12*(std::cos(theta1) - 1))*(std::cos(theta2) - wx22*(std::cos(theta2) - 1)))/2 + ((std::cos(theta1) - wy12*(std::cos(theta1) - 1))*(std::cos(theta2) - wy22*(std::cos(theta2) - 1)))/2 + ((std::cos(theta1) - wz12*(std::cos(theta1) - 1))*(std::cos(theta2) - wz22*(std::cos(theta2) - 1)))/2 - 1/2)));
    theta_d(3) = -(((wz1*std::cos(theta1) - wx1*wy1*std::sin(theta1))*(wz2*std::sin(theta2) + wx2*wy2*(std::cos(theta2) - 1)))/2 + ((wx1*std::cos(theta1) + wy1*wz1*std::sin(theta1))*(wx2*std::sin(theta2) - wy2*wz2*(std::cos(theta2) - 1)))/2 + ((wy1*std::cos(theta1) + wx1*wz1*std::sin(theta1))*(wy2*std::sin(theta2) - wx2*wz2*(std::cos(theta2) - 1)))/2 + ((wz1*std::cos(theta1) + wx1*wy1*std::sin(theta1))*(wz2*std::sin(theta2) - wx2*wy2*(std::cos(theta2) - 1)))/2 + ((wx1*std::cos(theta1) - wy1*wz1*std::sin(theta1))*(wx2*std::sin(theta2) + wy2*wz2*(std::cos(theta2) - 1)))/2 + ((wy1*std::cos(theta1) - wx1*wz1*std::sin(theta1))*(wy2*std::sin(theta2) + wx2*wz2*(std::cos(theta2) - 1)))/2 - ((- std::sin(theta1)*wx12 + std::sin(theta1))*((1 - std::cos(theta2))*wx22 + std::cos(theta2)))/2 - ((- std::sin(theta1)*wy12 + std::sin(theta1))*((1 - std::cos(theta2))*wy22 + std::cos(theta2)))/2 - ((- std::sin(theta1)*wz12 + std::sin(theta1))*((1 - std::cos(theta2))*wz22 + std::cos(theta2)))/2)/std::sqrt((1 - (((wz1*std::sin(theta1) + wx1*wy1*(std::cos(theta1) - 1))*(wz2*std::sin(theta2) + wx2*wy2*(std::cos(theta2) - 1)))/2 + ((wx1*std::sin(theta1) + wy1*wz1*(std::cos(theta1) - 1))*(wx2*std::sin(theta2) + wy2*wz2*(std::cos(theta2) - 1)))/2 + ((wy1*std::sin(theta1) + wx1*wz1*(std::cos(theta1) - 1))*(wy2*std::sin(theta2) + wx2*wz2*(std::cos(theta2) - 1)))/2 + ((wx1*std::sin(theta1) - wy1*wz1*(std::cos(theta1) - 1))*(wx2*std::sin(theta2) - wy2*wz2*(std::cos(theta2) - 1)))/2 + ((wy1*std::sin(theta1) - wx1*wz1*(std::cos(theta1) - 1))*(wy2*std::sin(theta2) - wx2*wz2*(std::cos(theta2) - 1)))/2 + ((wz1*std::sin(theta1) - wx1*wy1*(std::cos(theta1) - 1))*(wz2*std::sin(theta2) - wx2*wy2*(std::cos(theta2) - 1)))/2 + ((std::cos(theta1) - wx12*(std::cos(theta1) - 1))*(std::cos(theta2) - wx22*(std::cos(theta2) - 1)))/2 + ((std::cos(theta1) - wy12*(std::cos(theta1) - 1))*(std::cos(theta2) - wy22*(std::cos(theta2) - 1)))/2 + ((std::cos(theta1) - wz12*(std::cos(theta1) - 1))*(std::cos(theta2) - wz22*(std::cos(theta2) - 1)))/2 - 1/2)*(((wz1*std::sin(theta1) + wx1*wy1*(std::cos(theta1) - 1))*(wz2*std::sin(theta2) + wx2*wy2*(std::cos(theta2) - 1)))/2 + ((wx1*std::sin(theta1) + wy1*wz1*(std::cos(theta1) - 1))*(wx2*std::sin(theta2) + wy2*wz2*(std::cos(theta2) - 1)))/2 + ((wy1*std::sin(theta1) + wx1*wz1*(std::cos(theta1) - 1))*(wy2*std::sin(theta2) + wx2*wz2*(std::cos(theta2) - 1)))/2 + ((wx1*std::sin(theta1) - wy1*wz1*(std::cos(theta1) - 1))*(wx2*std::sin(theta2) - wy2*wz2*(std::cos(theta2) - 1)))/2 + ((wy1*std::sin(theta1) - wx1*wz1*(std::cos(theta1) - 1))*(wy2*std::sin(theta2) - wx2*wz2*(std::cos(theta2) - 1)))/2 + ((wz1*std::sin(theta1) - wx1*wy1*(std::cos(theta1) - 1))*(wz2*std::sin(theta2) - wx2*wy2*(std::cos(theta2) - 1)))/2 + ((std::cos(theta1) - wx12*(std::cos(theta1) - 1))*(std::cos(theta2) - wx22*(std::cos(theta2) - 1)))/2 + ((std::cos(theta1) - wy12*(std::cos(theta1) - 1))*(std::cos(theta2) - wy22*(std::cos(theta2) - 1)))/2 + ((std::cos(theta1) - wz12*(std::cos(theta1) - 1))*(std::cos(theta2) - wz22*(std::cos(theta2) - 1)))/2 - 1/2)));

    if(std::isnan(theta_d(0)))
        theta_d(0) = 0;
    if(std::isnan(theta_d(1)))
        theta_d(1) = 0;
    if(std::isnan(theta_d(2)))
        theta_d(2) = 0;
    if(std::isnan(theta_d(3)))
        theta_d(3) = 0;

    return theta_d;
}

inline Eigen::VectorXd applyTransform(Eigen::VectorXd transform, Eigen::VectorXd base)
{
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    Eigen::Matrix4d T;
    R = Eigen::AngleAxisd(transform(6), transform.segment(3,3));
    t = transform.segment(0,3);
    T.setIdentity();
    T.block(0,0,3,3) = R;
    T.block(0,3,3,1) = t;

    Eigen::Vector3d t_base;
    Eigen::Matrix3d R_base;
    Eigen::Matrix4d T_base;
    R_base = Eigen::AngleAxisd(base(6), base.segment(3,3));
    t_base = base.segment(0,3);
    T_base.setIdentity();
    T_base.block(0,0,3,3) = R_base;
    T_base.block(0,3,3,1) = t_base;

    Eigen::Matrix4d T_final;
    T_final = T*T_base;
    Eigen::VectorXd X_final;
    X_final.resize(7);
    X_final.segment(0,3) = T_final.block(0,3,3,1);
    Eigen::Matrix3d R_final = T_final.block(0,0,3,3);
    Eigen::AngleAxisd aa_final(R_final);
    X_final.segment(3,3) = aa_final.axis();
    X_final(6) = aa_final.angle();
    while(X_final(6) > M_PI) X_final(6) -= M_PI;

    return X_final;
}

inline Eigen::VectorXd applyTransformQuaternion(Eigen::VectorXd transform, Eigen::VectorXd base)
{
    Eigen::Matrix4d T = transformFromQuaternion(transform);

    Eigen::Matrix4d T_base = transformFromQuaternion(base);

    Eigen::Matrix4d T_final;
    T_final = T*T_base;

    return quaternionFromTransform(T_final);
}


inline double distanceAASS(Eigen::VectorXd aa1, Eigen::VectorXd aa2)
{
    rl::math::Transform t_1, t_2;
    t_1.matrix() = transformFromVector(aa1);
    t_2.matrix() = transformFromVector(aa2);
    rl::math::Vector delta_aa(6);
    rl::math::transform::toDelta(t_1, t_2, delta_aa);
//    delta_aa = t_1.toDelta(t_2);
    return delta_aa.dot(delta_aa);
}

inline Eigen::VectorXd distanceAA(Eigen::VectorXd aa1, Eigen::VectorXd aa2)
{
    rl::math::Transform t_1, t_2;
    t_1.matrix() = transformFromVector(aa1);
    t_2.matrix() = transformFromVector(aa2);
    rl::math::Vector delta_aa(6);
    rl::math::transform::toDelta(t_1, t_2, delta_aa);
//    delta_aa = t_1.toDelta(t_2);
    return delta_aa;
}

inline Eigen::VectorXd distanceTT(Eigen::Matrix4d t1, Eigen::Matrix4d t2)
{
    rl::math::Transform t_1, t_2;
    t_1.matrix() = t1;
    t_2.matrix() = t2;
    rl::math::Vector delta_aa(6);
    rl::math::transform::toDelta(t_1, t_2, delta_aa);
//    delta_aa = t_1.toDelta(t_2);
    return delta_aa;
}

inline double distanceTTSS(Eigen::Matrix4d t1, Eigen::Matrix4d t2)
{
    rl::math::Transform t_1, t_2;
    t_1.matrix() = t1;
    t_2.matrix() = t2;
    rl::math::Vector delta_aa(6);
    rl::math::transform::toDelta(t_1, t_2, delta_aa);
//    delta_aa = t_1.toDelta(t_2);
    return delta_aa.dot(delta_aa);
}

inline double distanceAADQ(Eigen::VectorXd aa1, Eigen::VectorXd aa2)
{
    rl::math::Transform t_1, t_2;
    t_1.matrix() = transformFromVector(aa1);
    t_2.matrix() = transformFromVector(aa2);
    double dist = rl::math::transform::distance<rl::math::Transform, rl::math::Transform, double> (t_1, t_2);
//    double dist = t_1.distance(t_2);
    return dist*dist;
}

inline double distanceDQ(Eigen::VectorXd q1, Eigen::VectorXd q2)
{
    rl::math::Transform t_1, t_2;
    t_1.matrix() = transformFromQuaternion(q1);
    t_2.matrix() = transformFromQuaternion(q2);
    double dist = rl::math::transform::distance<rl::math::Transform, rl::math::Transform, double> (t_1, t_2);
//    double dist = t_1.distance(t_2);
    return dist*dist;
}

inline Eigen::VectorXd getDelta(Eigen::VectorXd aa_target, Eigen::VectorXd aa_base)
{
    aa_target.segment(3,3) = aa_target.segment(3,3).normalized();
    aa_base.segment(3,3) = aa_base.segment(3,3).normalized();
    Eigen::Matrix4d t_target = transformFromVector(aa_target);
    Eigen::Matrix4d t_base = transformFromVector(aa_base);
    return vectorFromTransform(t_target*t_base.inverse());
}

inline Eigen::VectorXd getDeltaQuaternion(Eigen::VectorXd qt_target, Eigen::VectorXd qt_base)
{
    qt_target.segment(3,4) = qt_target.segment(3,4).normalized();
    qt_base.segment(3,4) = qt_base.segment(3,4).normalized();
    Eigen::Matrix4d t_target = transformFromQuaternion(qt_target);
    Eigen::Matrix4d t_base = transformFromQuaternion(qt_base);
    return quaternionFromTransform(t_target*t_base.inverse());
}

inline void calculateJacobian(rl::math::Matrix &J, int link_id, rl::math::Transform T, rl::mdl::Kinematic* kinematic)
{
    rl::math::Vector tmp(kinematic->getDof());

    for (std::size_t i = 0; i < kinematic->getDof(); ++i)
    {
        for (std::size_t j = 0; j < kinematic->getDof(); ++j)
        {
            tmp(j) = i == j ? 1 : 0;
        }

        kinematic->setVelocity(tmp);
        kinematic->forwardVelocity();

        J.block(0, i, 3, 1) = (T*kinematic->getBody(link_id)->t).linear() * (T.rotation()*(kinematic->getBody(link_id)->v.linear()+(kinematic->getBody(link_id)->v.angular()).cross(T.translation())));
        J.block(3, i, 3, 1) = (T*kinematic->getBody(link_id)->t).linear() * (T.rotation()*kinematic->getBody(link_id)->v.angular());
    }
}

inline void calculateJacobian(std::vector<rl::math::Matrix> &J, rl::mdl::Kinematic* &kinematic)
{
    rl::math::Vector tmp(kinematic->getDof());

    for (std::size_t jacobian_id = 0; jacobian_id < kinematic->getDof(); ++jacobian_id)
    {
        rl::math::Matrix J_i(kinematic->getOperationalDof() * 6, kinematic->getDof());
        for (std::size_t i = 0; i < kinematic->getDof(); ++i)
        {
            for (std::size_t j = 0; j < kinematic->getDof(); ++j)
            {
                tmp(j) = i == j ? 1 : 0;
            }

            kinematic->setVelocity(tmp);
            kinematic->forwardVelocity();

            J_i.block(0, i, 3, 1) = kinematic->getBody(jacobian_id)->t.linear() * kinematic->getBody(jacobian_id)->v.linear();
            J_i.block(3, i, 3, 1) = kinematic->getBody(jacobian_id)->t.linear() * kinematic->getBody(jacobian_id)->v.angular();
        }
        J.push_back(J_i);
    }
}

#endif // CONSTRAINTUTILS_H
