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


#include <QtGui>
#include "JoggingGUI.h"

#include "../src/ConstraintParser.h"
#include <json/json.h>
#include <json/value.h>
#include <json/reader.h>
#include <iostream>
#include <fstream>
#include <string>

class SkillJSON : public ConstraintSkill
{
public:
    SkillJSON(rl::mdl::Kinematic *kinematic, std::string json_string):
        ConstraintSkill(kinematic)
    {
        //JSON parsing
        std::stringstream errorMsg;
        std::vector<Eigen::Matrix4d> pose_vec;
        std::string solver_type;
        shape_robot = parser.parse_constraint_json(json_string, constraints, errorMsg, pose_vec, solver_type);
        if(solver_type == "aa" || solver_type == "quaternions")
        {
            skill_constraints.insert(skill_constraints.begin(), constraints.begin(), constraints.end());
        }
    }

    void updateSkillParams(Eigen::Matrix4d robotPose, Eigen::VectorXd q_current)
    {
        shape_robot->transform(robotPose);
        this->q_current = q_current;
    }
    ConstraintParser parser;

    std::vector<Constraint*> constraints;
    std::vector<Constraint*> constraints_env;
};

std::string coachIP = "localhost";
std::string kinematicFileURL = "scenario/comau-racer-7-14.mdl.xml";//scenario/universal-robots-ur5.mdl.xml
int modelId = 0;


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    JoggingGUI myGUI;

    std::string json_string;
    std::ifstream reader(argv[1], std::ifstream::binary);
    reader.seekg(0, std::ios::end);
    json_string.reserve(reader.tellg());
    reader.seekg(0, std::ios::beg);
    json_string.assign((std::istreambuf_iterator<char>(reader)), std::istreambuf_iterator<char>());

    ConstraintParser parser;
    std::stringstream errorMsg;
    RobotControl *controller = parser.parse_robot_controller_json(json_string, errorMsg);
    if(controller == nullptr)
    {
        std::cout << errorMsg.str() << std::endl;
        return -1;
    }
    controller->moveToHome();

    rl::math::Transform robotEFPose;
    controller->getEFPose(robotEFPose);

    SkillJSON *skill = new SkillJSON(controller->kinematic, json_string);

    CompositePrioritySkill *skill_cp;
    skill_cp = new CompositePrioritySkill(controller->kinematic, skill->shape_robot);
    skill_cp->addSkill(skill);
    std::string solver_type;
    bool env_constraints = parser.parse_environment_constraint_json(json_string, skill_cp, errorMsg, solver_type, controller->kinematic, skill->shape_robot);
    bool log = true;
    std::string log_fname = "log.txt";
    bool demo=false;
    bool exec=true;
//    demo = parser.parse_demo_params_json(json_string, errorMsg, log, log_fname, robot_move_axis, robot_move_iter, robot_move_value, obstacle_move_axis, obstacle_move_iter, obstacle_move_value, exec);
    std::vector<MoveSegment> traj;
    demo = parser.parse_demo_params_json(json_string, errorMsg, log, exec, log_fname, traj);
    SolverParams params;
    bool solver_params = parser.parse_solver_params_json(json_string, errorMsg, params);
    if(solver_params)
        skill_cp->setSolverParams(params);
    if(env_constraints)
    {
        int collision_constraint_id = -1;
        for(std::size_t env_constraint_id = 0;env_constraint_id < skill_cp->skill_constraints.size();++env_constraint_id)
        {
            if(skill_cp->skill_constraints[env_constraint_id]->constraint_name == "collision_constraint")
            {
                collision_constraint_id = env_constraint_id;
                break;
            }
        }
        if(collision_constraint_id != -1)
        {
            int model_id, body_id;
            if(parser.parse_collision_constraint_json(json_string, errorMsg, model_id, body_id))
            {
                CollisionConstraintPosition *constraint = dynamic_cast<CollisionConstraintPosition*>(skill_cp->skill_constraints[collision_constraint_id]);
                myGUI.setupObstacleDemo(controller, log_fname, std::string("SLSQP"), skill_cp, false, false, false, constraint->collision_bodies[0], model_id, body_id);
            }
        }
        else
            myGUI.setupDemo(controller, log_fname, std::string("SLSQP"), skill_cp, false, false);
    }
    else
    {
        myGUI.setupDemo(controller, log_fname, std::string("SLSQP"), skill_cp, false, false);
    }    

    if(demo)
    {
        myGUI.eval.noExec = !exec;

        for(std::size_t seg_id = 0;seg_id < traj.size();++seg_id)
        {
            Eigen::VectorXd userOffsets(6);
            userOffsets << 0,0,0,0,0,0;
            if(traj[seg_id].axis == 'x')
                userOffsets(0) = traj[seg_id].delta;
            else if(traj[seg_id].axis == 'y')
                userOffsets(1) = traj[seg_id].delta;
            else if(traj[seg_id].axis == 'z')
                userOffsets(2) = traj[seg_id].delta;
            else if(traj[seg_id].axis == 'a')
                userOffsets(3) = traj[seg_id].delta;
            else if(traj[seg_id].axis == 'b')
                userOffsets(4) = traj[seg_id].delta;
            else if(traj[seg_id].axis == 'c')
                userOffsets(5) = traj[seg_id].delta;

            if(traj[seg_id].object == "robot")
            {
                for(std::size_t move_id=0;move_id < traj[seg_id].iter;++move_id)
                {
                    myGUI.stepDemoMoveRobot(userOffsets);
                }
            }
            else if(traj[seg_id].object == "object")
            {
                for(std::size_t move_id=0;move_id < traj[seg_id].iter;++move_id)
                {
                    myGUI.stepDemoMoveObject(userOffsets);
                }
            }
        }
    }
    else
    {
        myGUI.setVisible(true);
        myGUI.show();
    }

    return a.exec();
}
