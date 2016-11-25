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


#ifndef CONSTRAINTPARSER_H
#define CONSTRAINTPARSER_H

#include <time.h>
#include "constraint/GeometricConstraints.h"
#include "constraint/RobotConstraints.h"
#include "constraint/ConstraintUtils.h"
#include "skill/CompositePrioritySkill.h"
#include "skill/CollisionAvoidanceSkill.h"
#include "RobotControl.h"
#include <jsoncpp/json/json.h>

struct MoveSegment
{
    std::string object;
    char axis;
    int iter;
    double delta;
};

class ConstraintParser
{
public:
    ConstraintParser();
    ObjectModel *parse_constraint_json(std::string constraints_json, std::vector<Constraint*> &constraints, std::stringstream &errorMsg, std::vector<Eigen::Matrix4d> &current_pose, std::string &solver_type);
    bool parse_environment_constraint_json(std::string constraints_json, CompositePrioritySkill* &skill_env, std::stringstream &errorMsg, std::string &solver_type, rl::mdl::Kinematic* &kinematic, ObjectModel* &shape_robot);
    bool parse_collision_constraint_json(std::string constraints_json, std::stringstream &errorMsg, int &model_id, int &body_id);
    RobotControl* parse_robot_controller_json(std::string constraints_json, std::stringstream &errorMsg);
    bool parse_demo_params_json(std::string constraints_json, std::stringstream &errorMsg, bool &log, bool &exec, std::string &log_fname, std::vector<MoveSegment> &traj);
    bool parse_solver_params_json(std::string constraints_json, std::stringstream &errorMsg, SolverParams &params);
    void addGeometricEntitiy(int rigid_object_id, PrimitiveShapeDescriptor* ps);
    int addObjectModel(std::string rigid_object);
    void addDependency(int base_object_id, int dependent_object_id);
    std::vector<ObjectModel*> rigid_objects;
    std::vector<std::pair<int, std::vector<int> > > dependencies; //(solved? 1:-1, list of dependent objects)
    int fixedObjectID;
private:


};

#endif // CONSTRAINTPARSER_H
