

#ifndef CONSTRAINTSKILL_H
#define CONSTRAINTSKILL_H

#include "../RobotControl.h"
#include "../solver/SolverNLOpt.h"
#include "../solver/SolverNLOptGenericKinematic.h"

class ConstraintSkill
{
public:
    ConstraintSkill();
    ConstraintSkill(rl::mdl::Kinematic* kinematic);
    virtual void updateSkillParams(Eigen::Matrix4d robotPose, Eigen::VectorXd q_current);
    virtual bool solveQNLOpt(Eigen::VectorXd &robotTargetConfiguration,std::string solver_name = "COBYLA");
    virtual bool solveDualQNLOpt(Eigen::VectorXd &robotTargetConfiguration);
    void setSolverParams(SolverParams &params);
    std::vector<Constraint*> skill_constraints;
    std::vector<Constraint*> skill_constraints_left;
    std::vector<Constraint*> skill_constraints_right;
    rl::mdl::Kinematic *kinematics;
    Eigen::VectorXd q_current;
    ObjectModel *shape_robot;
protected:
    std::vector<PrimitiveShapeDescriptor*> ps_vector;
    rl::mdl::Kinematic* kinematic1;
    rl::mdl::Kinematic* kinematic2;
    SolverParams solver_params;
};

#endif // CONSTRAINTSKILL_H
