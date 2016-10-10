

#include "ConstraintSkill.h"

ConstraintSkill::ConstraintSkill()
{
}

ConstraintSkill::ConstraintSkill(rl::mdl::Kinematic *kinematic):
    kinematics(kinematic)
{

}

void ConstraintSkill::updateSkillParams(Eigen::Matrix4d robotPose, Eigen::VectorXd q_current)
{
    this->q_current = q_current;
    shape_robot->transform(robotPose);
}

bool ConstraintSkill::solveQNLOpt(Eigen::VectorXd &robotTargetConfiguration, std::string solver_name)
{
    SolverNLOptGenericKinematic kin_solver_nlopt(skill_constraints, q_current, kinematics, solver_name);
    kin_solver_nlopt.setSolverParams(this->solver_params);
    robotTargetConfiguration = kin_solver_nlopt.solve();
//    KinematicSolverNLOpt kin_solver_nlopt(skill_constraints, q_current, kinematics);
//    robotTargetConfiguration = kin_solver_nlopt.solve();
    return true;
}


bool ConstraintSkill::solveDualQNLOpt(Eigen::VectorXd &robotTargetConfiguration)
{
    Eigen::VectorXd q1_current = q_current.segment(0,kinematic1->getDof());
    Eigen::VectorXd q2_current = q_current.segment(kinematic1->getDof(),kinematic2->getDof());
    DualArmKinematicSolverNLOpt solver_nlopt(skill_constraints, skill_constraints_left, skill_constraints_right, q1_current, q2_current, kinematic1, kinematic2);
    solver_nlopt.setSolverParams(this->solver_params);
    robotTargetConfiguration = solver_nlopt.solve();
    return true;
}

void ConstraintSkill::setSolverParams(SolverParams &params)
{
    this->solver_params = params;
}
