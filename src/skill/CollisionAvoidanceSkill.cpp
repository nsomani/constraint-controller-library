

#include "CollisionAvoidanceSkill.h"

CollisionAvoidanceSkill::CollisionAvoidanceSkill()
{
}

CollisionAvoidanceSkill::CollisionAvoidanceSkill(rl::mdl::Kinematic* &kinematic, std::shared_ptr< rl::sg::Scene > &collisionScene, ObjectModel* &robot, std::vector<rl::sg::Body *> &collision_bodies)
    : ConstraintSkill(kinematic), shape_scene(new ObjectModel())
{
    shape_robot = robot;
    constraint_collision = new CollisionConstraintPosition(kinematic);
    constraint_collision->collisionScene = collisionScene;
    constraint_collision->collision_bodies = collision_bodies;
    constraint_collision->priority_level = 1;
    skill_constraints.push_back(constraint_collision);
}

void CollisionAvoidanceSkill::updateSkillParams(Eigen::Matrix4d robotPose, Eigen::VectorXd q_current)
{
    shape_robot->transform(robotPose);
    this->q_current = q_current;
    constraint_collision->q_curr = q_current;
    constraint_collision->updateConstraintParams();
}

CollisionAvoidanceSkillExact::CollisionAvoidanceSkillExact()
{
}

CollisionAvoidanceSkillExact::CollisionAvoidanceSkillExact(rl::mdl::Kinematic* &kinematic, std::shared_ptr< rl::sg::Scene > &collisionScene, ObjectModel* &robot, std::vector<rl::sg::Body *> &collision_bodies)
    : ConstraintSkill(kinematic), shape_scene(new ObjectModel())
{
    shape_robot = robot;
    constraint_collision = new CollisionConstraintExact(kinematic);
    constraint_collision->collisionScene = collisionScene;
    constraint_collision->collision_bodies = collision_bodies;
    constraint_collision->priority_level = 1;
    skill_constraints.push_back(constraint_collision);
}

void CollisionAvoidanceSkillExact::updateSkillParams(Eigen::Matrix4d robotPose, Eigen::VectorXd q_current)
{
    shape_robot->transform(robotPose);
    this->q_current = q_current;
    constraint_collision->q_curr = q_current;
    constraint_collision->updateConstraintParams();
}
