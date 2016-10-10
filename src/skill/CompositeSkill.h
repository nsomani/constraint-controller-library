

#ifndef COMPOSITESKILL_H
#define COMPOSITESKILL_H

#include "ConstraintSkill.h"

class CompositeSkill : public ConstraintSkill
{
public:
    CompositeSkill();
    CompositeSkill(rl::mdl::Kinematic* &kinematic, ObjectModel* &robot);
    void addSkill(ConstraintSkill* skill);
    virtual void updateSkillParams(Eigen::Matrix4d robotPose, Eigen::VectorXd q_current);
protected:
    std::vector<ConstraintSkill*> skills;

};

#endif // COMPOSITESKILL_H
