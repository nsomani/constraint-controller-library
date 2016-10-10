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
#include "Evaluation.h"

Evaluation::Evaluation():
    log(true),
    noExec(false)
{

}

void Evaluation::setupDemo(RobotControl* &controller, std::string logger_fname, std::string solver_name, CompositePrioritySkill* &skill, bool offset_q, bool get_offset, bool plot)
{
    this->logger_fname = logger_fname;
    logger.open(logger_fname, std::ios_base::out);
    logger << "runtime ";
    for(std::size_t constraint_id = 0;constraint_id < skill->skill_constraints.size();++constraint_id)
    {
        logger << skill->skill_constraints[constraint_id]->constraint_name << " ";
    }
    for(std::size_t x_id = 0;x_id < 6;++x_id)
    {
        logger << x_id << " ";
    }
    for(std::size_t q_id = 0;q_id < controller->kinematic->getDof();++q_id)
    {
        logger << q_id << " ";
    }
    logger << "\n";
    this->controller = controller;
    this->solver_name = solver_name;
    this->skill = skill;
    this->offset_q = offset_q;
    this->get_offset = get_offset;
    this->plot = plot;
    this->controller->moveToHome();
}

void Evaluation::setupObstacleDemo(RobotControl* &controller, std::string logger_fname, std::string solver_name, CompositePrioritySkill* &skill, bool offset_q, bool get_offset, bool plot, rl::sg::Body *shape_body, int model_id, int body_id)
{
    this->logger_fname = logger_fname;
    logger.open(logger_fname, std::ios_base::out);
    logger << "runtime ";
    for(std::size_t constraint_id = 0;constraint_id < skill->skill_constraints.size();++constraint_id)
    {
        logger << skill->skill_constraints[constraint_id]->constraint_name << " ";
    }
    for(std::size_t x_id = 0;x_id < 6;++x_id)
    {
        logger << x_id << " ";
    }
    for(std::size_t q_id = 0;q_id < controller->kinematic->getDof();++q_id)
    {
        logger << q_id << " ";
    }
    logger << "\n";
    this->controller = controller;
    this->solver_name = solver_name;
    this->skill = skill;
    this->offset_q = offset_q;
    this->get_offset = get_offset;
    this->plot = plot;
    this->collision_body = shape_body;
    this->collision_model_id = model_id;
    this->collision_body_id = body_id;
    this->controller->moveToHome();
}

void Evaluation::stepDemoMoveRobot(Eigen::VectorXd userOffsets)
{

    rl::math::Transform robotEFPose;
    robotEFPose = controller->kinematic->getOperationalPosition(0);

    Eigen::VectorXd qd;
    qd.resize(controller->kinematic->getDof());
    Eigen::VectorXd q_offset;

    robotEFPose = controller->kinematic->getOperationalPosition(0);
    skill->updateSkillParams(robotEFPose.matrix(), controller->getRobotConfig());

    if(offset_q)
    {
        q_offset = controller->getRobotConfig();
        q_offset += userOffsets;
        controller->moveJoint(q_offset, true);
        //            controller->getEFPose(robotEFPose);
        robotEFPose = controller->kinematic->getOperationalPosition(0);
        skill->updateSkillParams(robotEFPose.matrix(), q_offset);
    }
    else
    {
        Eigen::Matrix4d userOffsetMat;
        Eigen::Matrix3d R_offset;
        R_offset = Eigen::AngleAxisd(userOffsets(5), Eigen::Vector3d::UnitZ())*
                Eigen::AngleAxisd(userOffsets(4), Eigen::Vector3d::UnitY())*
                Eigen::AngleAxisd(userOffsets(3), Eigen::Vector3d::UnitX());
        userOffsetMat.block(0,0,3,3) = R_offset;
        robotEFPose.matrix().block(0,0,3,3) = (robotEFPose.matrix().block(0,0,3,3))*R_offset;
        robotEFPose.translation().x() += userOffsets(0);
        robotEFPose.translation().y() += userOffsets(1);
        robotEFPose.translation().z() += userOffsets(2);
        controller->movePose(robotEFPose, true);
        robotEFPose = controller->kinematic->getOperationalPosition(0);
        skill->updateSkillParams(robotEFPose.matrix(), controller->getRobotConfig());
    }
    realtime::chrono::system_clock::time_point time_start = realtime::chrono::system_clock::now();
    skill->log = log;
    if(skill->solveQNLOpt(qd, solver_name))
    {
        realtime::chrono::system_clock::time_point time_end = realtime::chrono::system_clock::now();
        double runtime = std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_start).count();
        std::cout << "runtime: " << runtime << std::endl;
//        std::cout << qd.transpose() << std::endl;
        controller->moveJoint(qd, noExec);
        if(log)
        {
            robotEFPose = controller->kinematic->getOperationalPosition(0);
            Eigen::VectorXd constraintErrorVec = skill->constraintErrorVec;
            logger << runtime << " " << constraintErrorVec.transpose() << " " << quaternionFromTransform(robotEFPose.matrix()).transpose() << " " <<  qd.transpose() << "\n";
//            std::cout << runtime << " " << constraintErrorVec.transpose() << "\n";
        }
    }
    else
        std::cout << "solver failed" << std::endl;
}

void Evaluation::stepDemoMoveObject(Eigen::VectorXd userOffsets)
{

    rl::math::Transform body_frame;
    collision_body->getFrame(body_frame);

    Eigen::Matrix4d T_shape;
    T_shape.setIdentity();
    T_shape = body_frame.matrix();

    Eigen::Matrix4d userOffsetMat;
    Eigen::Matrix3d R_offset;
    R_offset = Eigen::AngleAxisd(userOffsets(5), Eigen::Vector3d::UnitZ())*
            Eigen::AngleAxisd(userOffsets(4), Eigen::Vector3d::UnitY())*
            Eigen::AngleAxisd(userOffsets(3), Eigen::Vector3d::UnitX());
    userOffsetMat = T_shape;
    userOffsetMat.block(0,0,3,3) = (userOffsetMat.block(0,0,3,3))*R_offset;
    userOffsetMat(0,3) += userOffsets(0);
    userOffsetMat(1,3) += userOffsets(1);
    userOffsetMat(2,3) += userOffsets(2);
    body_frame.matrix() = userOffsetMat;
    collision_body->setFrame(body_frame);

    T_shape.block(0,0,3,3) = (T_shape.block(0,0,3,3))*R_offset;
    T_shape(0,3) += userOffsets(0);
    T_shape(1,3) += userOffsets(1);
    T_shape(2,3) += userOffsets(2);

    if(!noExec)
    {
        Eigen::Matrix3d R = T_shape.block(0,0,3,3);
        std::ostringstream command;
        command << "echo 0 " << collision_model_id << " " << collision_body_id << " " << T_shape.block(0,3,3,1).transpose() << " " << R.eulerAngles(2,1,0).reverse().transpose() << " | nc " << controller->controller_ip << " 11235";
        std::system(command.str().c_str());
    }

    rl::math::Transform robotEFPose;
    robotEFPose = controller->kinematic->getOperationalPosition(0);

    Eigen::VectorXd qd;
    qd.resize(controller->kinematic->getDof());

    robotEFPose = controller->kinematic->getOperationalPosition(0);
    skill->updateSkillParams(robotEFPose.matrix(), controller->getRobotConfig());
    skill->log = log;

    realtime::chrono::system_clock::time_point time_start = realtime::chrono::system_clock::now();
    if(skill->solveQNLOpt(qd, solver_name))
    {
        realtime::chrono::system_clock::time_point time_end = realtime::chrono::system_clock::now();
        double runtime = std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_start).count();
        std::cout << "runtime: " << runtime << std::endl;
//        std::cout << qd.transpose() << std::endl;
        controller->moveJoint(qd, noExec);
        if(log)
        {
            robotEFPose = controller->kinematic->getOperationalPosition(0);
            Eigen::VectorXd constraintErrorVec = skill->constraintErrorVec;
            logger << runtime << " " << constraintErrorVec.transpose() << " " << quaternionFromTransform(robotEFPose.matrix()).transpose() << " " <<  qd.transpose() << "\n";
//             std::cout << runtime << " " << constraintErrorVec.transpose() << "\n";
        }
    }
    else
        std::cout << "solver failed" << std::endl;
}
