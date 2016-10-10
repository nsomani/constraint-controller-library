

#include "CompositeSkill.h"

CompositeSkill::CompositeSkill()
{
}

CompositeSkill::CompositeSkill(rl::mdl::Kinematic* &kinematic, ObjectModel* &robot):
    ConstraintSkill(kinematic)
{
    shape_robot = robot;
    skill_constraints.clear();
    skills.clear();
}

void CompositeSkill::addSkill(ConstraintSkill *skill)
{
    skills.push_back(skill);
    skill_constraints.insert(skill_constraints.end(), skill->skill_constraints.begin(), skill->skill_constraints.end());
}

void CompositeSkill::updateSkillParams(Eigen::Matrix4d robotPose, Eigen::VectorXd q_current)
{
    for(std::size_t skill_id = 0;skill_id < skills.size();++skill_id)
    {
        skills[skill_id]->updateSkillParams(robotPose, q_current);
    }
    this->q_current = q_current;
}
