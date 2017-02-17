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
