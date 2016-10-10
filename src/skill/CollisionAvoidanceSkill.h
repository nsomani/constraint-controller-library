

#ifndef COLLISIONAVOIDANCESKILL_H
#define COLLISIONAVOIDANCESKILL_H

#include "ConstraintSkill.h"

class CollisionAvoidanceSkill : public ConstraintSkill
{
public:
    CollisionAvoidanceSkill();
    CollisionAvoidanceSkill(rl::mdl::Kinematic* &kinematic, std::shared_ptr< rl::sg::Scene > &collisionScene, ObjectModel* &robot, std::vector<rl::sg::Body*> &collision_bodies);
    void updateSkillParams(Eigen::Matrix4d robotPose, Eigen::VectorXd q_current);
    ObjectModel *shape_scene;
    CollisionConstraintPosition *constraint_collision;
private:
    void updatePSPose(PrimitiveShapeDescriptor *ps, Eigen::Matrix4d pose);

};

class CollisionAvoidanceSkillExact : public ConstraintSkill
{
public:
    CollisionAvoidanceSkillExact();
    CollisionAvoidanceSkillExact(rl::mdl::Kinematic* &kinematic, std::shared_ptr< rl::sg::Scene > &collisionScene, ObjectModel* &robot, std::vector<rl::sg::Body*> &collision_bodies);
    void updateSkillParams(Eigen::Matrix4d robotPose, Eigen::VectorXd q_current);
    ObjectModel *shape_scene;
    CollisionConstraintExact *constraint_collision;
private:
    void updatePSPose(PrimitiveShapeDescriptor *ps, Eigen::Matrix4d pose);

};

#endif // COLLISIONAVOIDANCESKILL_H
