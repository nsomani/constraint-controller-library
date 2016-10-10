

#ifndef COMPOSITEPRIORITYSKILL_H
#define COMPOSITEPRIORITYSKILL_H
#include "CompositeSkill.h"

class CompositePrioritySkill : public CompositeSkill
{
public:
    CompositePrioritySkill();
    CompositePrioritySkill(rl::mdl::Kinematic* &kinematic, ObjectModel* &robot);
    bool solveQNLOpt(Eigen::VectorXd &robotTargetConfiguration);
    bool solveQNLOpt(Eigen::VectorXd &robotTargetConfiguration, std::string solver_name);
    bool solveTwoArmQNLOpt(Eigen::VectorXd &robotTargetConfiguration);
    bool solveDualQNLOpt(Eigen::VectorXd &robotTargetConfiguration);    
    bool log;
    Eigen::VectorXd constraintErrorVec;
};

class CylindricalObjectTopGraspSkill : public ConstraintSkill
{
public:
    CylindricalObjectTopGraspSkill(rl::mdl::Kinematic *kinematic, ObjectModel *robot, CylinderShapeDescriptor* &cylinder, float gripper_span_limit = 0.1, float gripper_finger_length = 0.001)
        : ConstraintSkill(kinematic), shape_scene(new ObjectModel())
    {
        shape_robot = robot;
        PlaneShapeDescriptor *cylinder_center_plane = new PlaneShapeDescriptor(cylinder->center, cylinder->axis);
        LineShapeDescriptor *cylinder_center_line = new LineShapeDescriptor(cylinder->center, cylinder->axis);
        PointShapeDescriptor *cylinder_center_point = cylinder->center;

        shape_scene->primitive_shapes.push_back(cylinder);
        shape_scene->primitive_shapes.push_back(cylinder_center_plane);
        shape_scene->primitive_shapes.push_back(cylinder_center_point);
        shape_scene->primitive_shapes.push_back(cylinder_center_line);

        CylinderShapeDescriptor *gripper_virtual_cylinder = new CylinderShapeDescriptor(Eigen::Vector3d(0,0,0.2-(gripper_finger_length/2)), Eigen::Vector3d(0,0,1), gripper_span_limit, gripper_finger_length);
        PointShapeDescriptor *gripper_point = new PointShapeDescriptor(Eigen::Vector3d(0,0,0.2));
        PlaneShapeDescriptor *gripper_bottom_plane = new PlaneShapeDescriptor(Eigen::Vector3d(0,0,0.2), Eigen::Vector3d(0,0,1));
        PlaneShapeDescriptor *gripper_face_plane = new PlaneShapeDescriptor(Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,1,0));
        PlaneShapeDescriptor *gripper_side_plane = new PlaneShapeDescriptor(Eigen::Vector3d(0,0,0), Eigen::Vector3d(1,0,0));
        PointShapeDescriptor *gripper_finger_point_1 = new PointShapeDescriptor(Eigen::Vector3d(gripper_span_limit/2,0,0.2));
        PointShapeDescriptor *gripper_finger_point_2 = new PointShapeDescriptor(Eigen::Vector3d(-gripper_span_limit/2,0,0.2));
        shape_robot->primitive_shapes.push_back(gripper_virtual_cylinder);
        shape_robot->primitive_shapes.push_back(gripper_point);
        shape_robot->primitive_shapes.push_back(gripper_face_plane);
        shape_robot->primitive_shapes.push_back(gripper_bottom_plane);
        shape_robot->primitive_shapes.push_back(gripper_side_plane);
        shape_robot->primitive_shapes.push_back(gripper_finger_point_1);
        shape_robot->primitive_shapes.push_back(gripper_finger_point_2);

        PlanePlaneDistanceMinMaxConstraint *constraintPlane = new PlanePlaneDistanceMinMaxConstraint(cylinder_center_plane, gripper_bottom_plane, -1*cylinder->height/2, cylinder->height/2);
        skill_constraints.push_back(constraintPlane);

        CylinderCylinderParallelDistanceMinMaxConstraint *constraintCylinder = new CylinderCylinderParallelDistanceMinMaxConstraint(cylinder, gripper_virtual_cylinder, ((gripper_finger_length/2)-cylinder->radius), (gripper_finger_length/2));
        skill_constraints.push_back(constraintCylinder);

        constraint_joint_velocity = new RobotJointVelocityLimitconstraint(kinematic);
        constraint_joint_velocity->priority_level = 1;
//        skill_constraints.push_back(constraint_joint_velocity);

        constraint_vel = new RobotVelocityLimitconstraint(kinematic);
//        skill_constraints.push_back(constraint_vel);


    }
    void updateSkillParams(Eigen::Matrix4d robotPose, Eigen::VectorXd q_current)
    {
        shape_robot->transform(robotPose);
        this->q_current = q_current;
        constraint_joint_velocity->q_curr = q_current;
        constraint_vel->EFpos_curr = robotPose;
        constraint_vel->updateConstraintParams();
    }

    ObjectModel *shape_scene;
    RobotDesiredPositionConstraint *constraint_position;
    RobotJointVelocityLimitconstraint *constraint_joint_velocity;
    RobotVelocityLimitconstraint *constraint_vel;

};

#endif // COMPOSITEPRIORITYSKILL_H
