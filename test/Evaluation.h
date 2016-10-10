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
#ifndef EVALUATION_H
#define EVALUATION_H

#include "../src/RobotControl.h"
#include "../src/skill/ConstraintSkill.h"
#include "../src/skill/CompositePrioritySkill.h"
#include "../src/skill/CollisionAvoidanceSkill.h"
#include <time.h>
#include <thread>
#include <fstream>
namespace realtime = std;

class Evaluation
{
public:
    Evaluation();
    void setupDemo(RobotControl* &controller, std::string logger_fname, std::string solver_name, CompositePrioritySkill* &skill, bool offset_q, bool get_offset, bool plot=false);
    void setupObstacleDemo(RobotControl* &controller, std::string logger_fname, std::string solver_name, CompositePrioritySkill* &skill, bool offset_q, bool get_offset, bool plot, rl::sg::Body *shape_body, int model_id, int body_id);
    void stepDemoMoveRobot(Eigen::VectorXd userOffsets);
    void stepDemoMoveObject(Eigen::VectorXd userOffsets);
    bool noExec;
    bool log;

private:
    RobotControl* controller;
    std::string logger_fname;
    std::fstream logger;
    std::string solver_name;
    CompositePrioritySkill *skill;
    bool offset_q;
    bool get_offset;
    bool plot;
    rl::sg::Body* collision_body;
    int collision_model_id;
    int collision_body_id;
};

#endif // EVALUATION_H
