

#ifndef GEOMETRICCONSTRAINTS_H
#define GEOMETRICCONSTRAINTS_H

#include "Constraint.h"

class PlanePlaneDistanceMinMaxConstraint : public OperationalPositionConstraint
{
public:
    PlanePlaneDistanceMinMaxConstraint();
    PlanePlaneDistanceMinMaxConstraint(PlaneShapeDescriptor *fixed, PlaneShapeDescriptor *constrained, double d_min, double d_max, bool maximize_d = false, bool minimize_d = false);
    double distance_min, distance_max;
    bool unsigned_distance;
    bool minimize_distance;
    bool maximize_distance;
    virtual int getNumConstraints();
    virtual Eigen::VectorXd calculateConstraintValue(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivative(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd calculateConstraintValueAA(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivativeAA(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd getLowerBounds();
    virtual Eigen::VectorXd getUpperBounds();
    PlaneShapeDescriptor *plane_fixed;
    PlaneShapeDescriptor *plane_constrained;
};

class PlanePlaneDistanceAngleMinMaxConstraint : public OperationalPositionConstraint
{
public:
    PlanePlaneDistanceAngleMinMaxConstraint();
    PlanePlaneDistanceAngleMinMaxConstraint(PlaneShapeDescriptor *fixed, PlaneShapeDescriptor *constrained, double d_min, double d_max, double a_min, double a_max, bool maximize_d = false, bool minimize_d = false);
    double distance_min, distance_max;
    double angle_min, angle_max;
    bool unsigned_distance;
    bool minimize_distance;
    bool maximize_distance;
    virtual int getNumConstraints();
    virtual Eigen::VectorXd calculateConstraintValue(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivative(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd calculateConstraintValueAA(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivativeAA(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd getLowerBounds();
    virtual Eigen::VectorXd getUpperBounds();
    PlaneShapeDescriptor *plane_fixed;
    PlaneShapeDescriptor *plane_constrained;
};

class PlanePlaneDistanceConstraint : public PlanePlaneDistanceMinMaxConstraint
{
public:
    PlanePlaneDistanceConstraint();
    PlanePlaneDistanceConstraint(PlaneShapeDescriptor *fixed, PlaneShapeDescriptor *constrained, double d, bool maximize_d=false, bool minimize_d=false);
    double distance;
};

class PlanePlaneCoincidentConstraint : public PlanePlaneDistanceConstraint
{
public:
    PlanePlaneCoincidentConstraint(PlaneShapeDescriptor *fixed, PlaneShapeDescriptor *constrained);
};

class PlanePlaneSideConstraint : public PlanePlaneDistanceMinMaxConstraint
{
public:
    PlanePlaneSideConstraint();
    PlanePlaneSideConstraint(PlaneShapeDescriptor *fixed, PlaneShapeDescriptor *constrained);
};

class PlanePlaneAngleMinMaxConstraint : public OperationalPositionConstraint
{
public:
    PlanePlaneAngleMinMaxConstraint();
    PlanePlaneAngleMinMaxConstraint(PlaneShapeDescriptor *fixed, PlaneShapeDescriptor *constrained, double a_min, double a_max, bool maximize_a = false, bool minimize_a = false);
    double angle_min, angle_max;
    bool unsigned_angle;
    bool minimize_angle;
    bool maximize_angle;
    virtual int getNumConstraints();
    virtual Eigen::VectorXd calculateConstraintValue(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivative(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd calculateConstraintValueAA(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivativeAA(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd getLowerBounds();
    virtual Eigen::VectorXd getUpperBounds();
    PlaneShapeDescriptor *plane_fixed;
    PlaneShapeDescriptor *plane_constrained;
};

class PlanePlaneAngleConstraint : public PlanePlaneAngleMinMaxConstraint
{
public:
    PlanePlaneAngleConstraint(PlaneShapeDescriptor *fixed, PlaneShapeDescriptor *constrained, double a);
    double angle;
};


class PlanePlaneParallelConstraint : public PlanePlaneAngleMinMaxConstraint
{
public:
    PlanePlaneParallelConstraint();
    PlanePlaneParallelConstraint(PlaneShapeDescriptor *fixed, PlaneShapeDescriptor *constrained);

};

class PointPlaneDistanceMinMaxConstraint : public OperationalPositionConstraint
{
public:
    PointPlaneDistanceMinMaxConstraint();
    PointPlaneDistanceMinMaxConstraint(PointShapeDescriptor *fixed, PlaneShapeDescriptor *constrained, double d_min, double d_max, bool maximize_d = false, bool minimize_d = false);
    double distance_min, distance_max;
    bool unsigned_distance;
    bool minimize_distance;
    bool maximize_distance;
    virtual int getNumConstraints();
    virtual Eigen::VectorXd calculateConstraintValue(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivative(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd calculateConstraintValueAA(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivativeAA(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd getLowerBounds();
    virtual Eigen::VectorXd getUpperBounds();
    PointShapeDescriptor *point_fixed;
    PlaneShapeDescriptor *plane_constrained;

};

class PlanePointDistanceMinMaxConstraint : public OperationalPositionConstraint
{
public:
    PlanePointDistanceMinMaxConstraint();
    PlanePointDistanceMinMaxConstraint(PlaneShapeDescriptor *fixed, PointShapeDescriptor *constrained, double d_min, double d_max, bool maximize_d = false, bool minimize_d = false);
    double distance_min, distance_max;
    bool unsigned_distance;
    bool minimize_distance;
    bool maximize_distance;
    virtual int getNumConstraints();
    virtual Eigen::VectorXd calculateConstraintValue(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivative(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd calculateConstraintValueAA(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivativeAA(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd getLowerBounds();
    virtual Eigen::VectorXd getUpperBounds();
    PointShapeDescriptor *point_constrained;
    PlaneShapeDescriptor *plane_fixed;

};

class PlanePointDistanceConstraint: public PlanePointDistanceMinMaxConstraint
{
public:
    PlanePointDistanceConstraint();
    PlanePointDistanceConstraint(PlaneShapeDescriptor *fixed, PointShapeDescriptor *constrained, double d);
    double distance;
};

class PlanePointCoincidentConstraint: public PlanePointDistanceMinMaxConstraint
{
public:
    PlanePointCoincidentConstraint();
    PlanePointCoincidentConstraint(PlaneShapeDescriptor *fixed, PointShapeDescriptor *constrained);
};

class PointPlaneCoincidentConstraint: public PointPlaneDistanceMinMaxConstraint
{
public:
    PointPlaneCoincidentConstraint();
    PointPlaneCoincidentConstraint(PointShapeDescriptor *fixed, PlaneShapeDescriptor *constrained);
};

class CylinderCylinderParallelDistanceMinMaxConstraint : public OperationalPositionConstraint
{
public:
    CylinderCylinderParallelDistanceMinMaxConstraint();
    CylinderCylinderParallelDistanceMinMaxConstraint(CylinderShapeDescriptor *fixed, CylinderShapeDescriptor *constrained, double d_min, double d_max);
    double distance_min, distance_max;
    virtual int getNumConstraints();
    virtual Eigen::VectorXd calculateConstraintValue(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivative(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd calculateConstraintValueAA(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivativeAA(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd getLowerBounds();
    virtual Eigen::VectorXd getUpperBounds();
    CylinderShapeDescriptor *cylinder_fixed;
    CylinderShapeDescriptor *cylinder_constrained;
};

class CylinderCylinderParallelDistanceConstraint : public CylinderCylinderParallelDistanceMinMaxConstraint
{
public:
    CylinderCylinderParallelDistanceConstraint(CylinderShapeDescriptor *fixed, CylinderShapeDescriptor *constrained, double dist);
};

class CylinderCylinderConcentricConstraint : public CylinderCylinderParallelDistanceMinMaxConstraint
{
public:
    CylinderCylinderConcentricConstraint(CylinderShapeDescriptor *fixed, CylinderShapeDescriptor *constrained);
};

class CylinderCylinderAngleMinMaxConstraint : public OperationalPositionConstraint
{
public:
    CylinderCylinderAngleMinMaxConstraint(CylinderShapeDescriptor *fixed, CylinderShapeDescriptor *constrained, double a_min, double a_max);
    double angle_min, angle_max;
    virtual int getNumConstraints();
    virtual Eigen::VectorXd calculateConstraintValue(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivative(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd calculateConstraintValueAA(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivativeAA(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd getLowerBounds();
    virtual Eigen::VectorXd getUpperBounds();
    CylinderShapeDescriptor *cylinder_fixed;
    CylinderShapeDescriptor *cylinder_constrained;
};

class CylinderCylinderAngleConstraint : public CylinderCylinderAngleMinMaxConstraint
{
public:
    CylinderCylinderAngleConstraint(CylinderShapeDescriptor *fixed, CylinderShapeDescriptor *constrained, double a);
    double angle;
};


class CylinderCylinderParallelConstraint : public CylinderCylinderAngleConstraint
{
public:
    CylinderCylinderParallelConstraint(CylinderShapeDescriptor *fixed, CylinderShapeDescriptor *constrained);
private:
    PrimitiveShapeDescriptor *translation_manifold;
};

class CylinderCylinderParallelInsideConstraint : public CylinderCylinderParallelDistanceMinMaxConstraint
{
public:
    CylinderCylinderParallelInsideConstraint(CylinderShapeDescriptor *fixed, CylinderShapeDescriptor *constrained);
};

class ConeConeParallelDistanceMinMaxConstraint : public OperationalPositionConstraint
{
public:
    ConeConeParallelDistanceMinMaxConstraint(ConeShapeDescriptor *fixed, ConeShapeDescriptor *constrained, double d_min, double d_max);
    double distance_min, distance_max;
    virtual int getNumConstraints();
    virtual Eigen::VectorXd calculateConstraintValue(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivative(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd calculateConstraintValueAA(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivativeAA(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd getLowerBounds();
    virtual Eigen::VectorXd getUpperBounds();
    ConeShapeDescriptor *cone_fixed;
    ConeShapeDescriptor *cone_constrained;

};

class LinePointDistanceMinMaxConstraint : public OperationalPositionConstraint
{
public:
    LinePointDistanceMinMaxConstraint();
    LinePointDistanceMinMaxConstraint(LineShapeDescriptor *fixed, PointShapeDescriptor *constrained, double d_min, double d_max, bool maximize_d = false, bool minimize_d = false);
    double distance_min, distance_max;
    bool unsigned_distance;
    bool minimize_distance;
    bool maximize_distance;
    virtual int getNumConstraints();
    virtual Eigen::VectorXd calculateConstraintValue(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivative(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd calculateConstraintValueAA(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivativeAA(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd getLowerBounds();
    virtual Eigen::VectorXd getUpperBounds();
    LineShapeDescriptor *line_fixed;
    PointShapeDescriptor *point_constrained;
};

class LinePointDistanceConstraint : public LinePointDistanceMinMaxConstraint
{
public:
    LinePointDistanceConstraint(LineShapeDescriptor *fixed, PointShapeDescriptor *constrained, double d);
    double distance;
};

class LinePointCoincidentConstraint : public LinePointDistanceConstraint
{
public:
    LinePointCoincidentConstraint(LineShapeDescriptor *fixed, PointShapeDescriptor *constrained);
    virtual int getNumConstraints();
    virtual Eigen::VectorXd calculateConstraintValue(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivative(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd calculateConstraintValueAA(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivativeAA(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd getLowerBounds();
    virtual Eigen::VectorXd getUpperBounds();
};

class PointLineCoincidentConstraint : public OperationalPositionConstraint
{
public:
    PointLineCoincidentConstraint(PointShapeDescriptor *fixed, LineShapeDescriptor *constrained);
    virtual int getNumConstraints();
    virtual Eigen::VectorXd calculateConstraintValue(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivative(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd calculateConstraintValueAA(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivativeAA(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd getLowerBounds();
    virtual Eigen::VectorXd getUpperBounds();
    PointShapeDescriptor *point_fixed;
    LineShapeDescriptor *line_constrained;
};

class PointLineDistanceMinMaxConstraint : public OperationalPositionConstraint
{
public:
    PointLineDistanceMinMaxConstraint(PointShapeDescriptor *fixed, LineShapeDescriptor *constrained, double d_min, double d_max, bool maximize_d = false, bool minimize_d = false);
    double distance_min, distance_max;
    bool unsigned_distance;
    bool minimize_distance;
    bool maximize_distance;
    virtual int getNumConstraints();
    virtual Eigen::VectorXd calculateConstraintValue(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivative(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd getLowerBounds();
    virtual Eigen::VectorXd calculateConstraintValueAA(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivativeAA(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd getUpperBounds();
    PointShapeDescriptor *point_fixed;
    LineShapeDescriptor *line_constrained;
};

class PointLineDistanceConstraint : public PointLineDistanceMinMaxConstraint
{
public:
    PointLineDistanceConstraint(PointShapeDescriptor *fixed, LineShapeDescriptor *constrained, double d);
    double distance;
    PointShapeDescriptor *point_fixed;
    LineShapeDescriptor *line_constrained;
};


class LineLineParallelDistanceMinMaxConstraint: public OperationalPositionConstraint
{
public:
    LineLineParallelDistanceMinMaxConstraint(LineShapeDescriptor *fixed, LineShapeDescriptor *constrained, double d_min, double d_max, bool maximize_d = false, bool minimize_d = false);
    double distance_min, distance_max;
    bool unsigned_distance;
    bool minimize_distance;
    bool maximize_distance;
    virtual int getNumConstraints();
    virtual Eigen::VectorXd calculateConstraintValue(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivative(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd calculateConstraintValueAA(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivativeAA(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd getLowerBounds();
    virtual Eigen::VectorXd getUpperBounds();
    LineShapeDescriptor *line_fixed;
    LineShapeDescriptor *line_constrained;
};

class LineLineParallelDistanceConstraint: public LineLineParallelDistanceMinMaxConstraint
{
public:
    LineLineParallelDistanceConstraint(LineShapeDescriptor *fixed, LineShapeDescriptor *constrained, double d);
    double distance;
};

class LineLineParallelConstraint: public OperationalPositionConstraint
{
public:
    LineLineParallelConstraint(LineShapeDescriptor *fixed, LineShapeDescriptor *constrained);
    virtual int getNumConstraints();
    virtual Eigen::VectorXd calculateConstraintValue(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivative(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd calculateConstraintValueAA(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivativeAA(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd getLowerBounds();
    virtual Eigen::VectorXd getUpperBounds();
    LineShapeDescriptor *line_fixed;
    LineShapeDescriptor *line_constrained;
};

class LineLineCoincidentConstraint: public LineLineParallelDistanceConstraint
{
public:
    LineLineCoincidentConstraint(LineShapeDescriptor *fixed, LineShapeDescriptor *constrained);
};

class LineLineAngleConstraint: public OperationalPositionConstraint
{
public:
    LineLineAngleConstraint(LineShapeDescriptor *fixed, LineShapeDescriptor *constrained, double angle);
    double angle;
    virtual int getNumConstraints();
    virtual Eigen::VectorXd calculateConstraintValue(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivative(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd calculateConstraintValueAA(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivativeAA(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd getLowerBounds();
    virtual Eigen::VectorXd getUpperBounds();
    LineShapeDescriptor *line_fixed;
    LineShapeDescriptor *line_constrained;
};

class PlaneLineParallelConstraint: public OperationalPositionConstraint
{
public:
    PlaneLineParallelConstraint(PlaneShapeDescriptor *fixed, LineShapeDescriptor *constrained);
    virtual int getNumConstraints();
    virtual Eigen::VectorXd calculateConstraintValue(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivative(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd calculateConstraintValueAA(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivativeAA(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd getLowerBounds();
    virtual Eigen::VectorXd getUpperBounds();
    PlaneShapeDescriptor *plane_fixed;
    LineShapeDescriptor *line_constrained;
};

class PlaneLineAngleConstraint: public OperationalPositionConstraint
{
public:
    PlaneLineAngleConstraint(PlaneShapeDescriptor *fixed, LineShapeDescriptor *constrained, double a);
    virtual int getNumConstraints();
    virtual Eigen::VectorXd calculateConstraintValue(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivative(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd calculateConstraintValueAA(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivativeAA(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd getLowerBounds();
    virtual Eigen::VectorXd getUpperBounds();
    PlaneShapeDescriptor *plane_fixed;
    LineShapeDescriptor *line_constrained;
    double angle;
};

class PlaneLinePerpendicularDistanceConstraint: public OperationalPositionConstraint
{
public:
    double distance;
    PlaneLinePerpendicularDistanceConstraint(PlaneShapeDescriptor *fixed, LineShapeDescriptor *constrained, double d);
    virtual int getNumConstraints();
    virtual Eigen::VectorXd calculateConstraintValue(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivative(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd calculateConstraintValueAA(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivativeAA(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd getLowerBounds();
    virtual Eigen::VectorXd getUpperBounds();
    PlaneShapeDescriptor *plane_fixed;
    LineShapeDescriptor *line_constrained;
};

class PointPointDistanceMinMaxConstraint: public OperationalPositionConstraint
{
public:
    PointPointDistanceMinMaxConstraint(PointShapeDescriptor *fixed, PointShapeDescriptor *constrained, double d_min, double d_max);
    double distance_min, distance_max;
    virtual int getNumConstraints();
    virtual Eigen::VectorXd calculateConstraintValue(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivative(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd calculateConstraintValueAA(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivativeAA(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd getLowerBounds();
    virtual Eigen::VectorXd getUpperBounds();
    PointShapeDescriptor *point_fixed;
    PointShapeDescriptor *point_constrained;
};

class PointPointDistanceConstraint: public PointPointDistanceMinMaxConstraint
{
public:
    PointPointDistanceConstraint(PointShapeDescriptor *fixed, PointShapeDescriptor *constrained, double d);
    double distance;
    PointShapeDescriptor *point_fixed;
    PointShapeDescriptor *point_constrained;
};


class PointPointCoincidentConstraint: public PointPointDistanceMinMaxConstraint
{
public:
    PointPointCoincidentConstraint(PointShapeDescriptor *fixed, PointShapeDescriptor *constrained);
};

class PlanePlaneSymmetryConstraint: public OperationalPositionConstraint
{
public:
    PlanePlaneSymmetryConstraint(PlaneShapeDescriptor *fixed1, PlaneShapeDescriptor *fixed2, PlaneShapeDescriptor *constrained1, PlaneShapeDescriptor *constrained2);
    double distance_min, distance_max;
    virtual int getNumConstraints();
    virtual Eigen::VectorXd calculateConstraintValue(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivative(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd calculateConstraintValueAA(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivativeAA(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd getLowerBounds();
    virtual Eigen::VectorXd getUpperBounds();

    PlaneShapeDescriptor *plane_fixed1, *plane_fixed2;
    PlaneShapeDescriptor *plane_constrained1, *plane_constrained2;
};

class VectorVectorAngleConstraint: public OperationalPositionConstraint
{
public:
    VectorVectorAngleConstraint(VectorShapeDescriptor *fixed, VectorShapeDescriptor *constrained, double angle);
    virtual int getNumConstraints();
    virtual Eigen::VectorXd calculateConstraintValue(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivative(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd calculateConstraintValueAA(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivativeAA(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd getLowerBounds();
    virtual Eigen::VectorXd getUpperBounds();
    double angle;
    VectorShapeDescriptor *vector_fixed;
    VectorShapeDescriptor *vector_constrained;
};

class LineSegmentPointCoincidentConstraint: public OperationalPositionConstraint
{
public:
    LineSegmentPointCoincidentConstraint(LineSegmentShapeDescriptor *fixed, PointShapeDescriptor *constrained);
    virtual int getNumConstraints();
    virtual Eigen::VectorXd calculateConstraintValue(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivative(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd calculateConstraintValueAA(Eigen::VectorXd &vec);
    virtual Eigen::MatrixXd calculateConstraintDerivativeAA(Eigen::VectorXd &vec);
    virtual Eigen::VectorXd getLowerBounds();
    virtual Eigen::VectorXd getUpperBounds();
    double angle;
    LineSegmentShapeDescriptor *linesegment_fixed;
    PointShapeDescriptor *point_constrained;
};

class VectorVectorCoincidentConstraint: public VectorVectorAngleConstraint
{
public:
    VectorVectorCoincidentConstraint(VectorShapeDescriptor *fixed, VectorShapeDescriptor *constrained);
};

#endif // GEOMETRICCONSTRAINTS_H

