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


#include "RobotControl.h"
#include <fstream>
#include <sstream>
#include <QFile>
#include <QTextStream>

RobotControl::RobotControl(const std::string& kinematics, const std::string& robot_name, const int& workcell_model_id, const std::string &coach_ip, int dof) :
    model(),
    q(),
    qp(),
    controller_ip(coach_ip)
{
    std::cout << "robot_name " << robot_name << std::endl;
    std::cout << "workcell_model_id " << workcell_model_id << std::endl;

#ifdef COACH
    controller = new rl::hal::Coach(dof, 0.007110f, workcell_model_id, coach_ip);
#endif
    this->controller->open();
    this->controller->start();

    robotSemanticName=robot_name;

    QString robot_name_string(robot_name.c_str());
    std::stringstream robot_name_id;
    if(robot_name_string.contains("comau"))
    {
        robot_name_id  << "comau_" << workcell_model_id;
    }
    else if(robot_name_string.contains("kuka"))
    {
        robot_name_id  << "kuka_" << workcell_model_id;
    }
    else if(robot_name_string.contains("guedel-portico-light"))
    {
        robot_name_id  << "comau_" << workcell_model_id;
    }
    else if(robot_name_string.contains("lwr"))
    {
        robot_name_id  << "lwr_" << workcell_model_id;
    }
    else if(robot_name_string.contains("ur5"))
    {
        robot_name_id  << "ur5_" << workcell_model_id;
    }
    else
    {
        std::cout << "robot type " << robot_name << " " << "not defined and does not contain string subset comau, kuka or guedel_portico_light" << std::endl;
        throw;
    }

    robotName=robot_name_id.str();

    rl::mdl::XmlFactory factory;
    this->model.reset(factory.create(kinematics));

    this->kinematic = dynamic_cast< rl::mdl::Kinematic* >(this->model.get());

#ifdef COACH
    this->q.resize(this->controller->getDof());
    this->qp.resize(this->controller->getDof());
    this->qpp.resize(this->controller->getDof());
    qp.setZero();
    qpp.setZero();
    if(robot_name_string.contains("guedel-portico-light"))
    {
        //this->q.setZero();
        q(0)=0.0f;
        q(1)=0.0f;
        q(2)=0.0f;
        q(3)=0.0f;
        q(4)=0.0f;
        q(5)=0.0f;
    }
    else if (robot_name_string.contains("lwr"))
    {
        q(0)=0.0f;
        q(1)=0.0f;
        q(2)=(170.0/180)*M_PI;
        q(3)=M_PI_2;
        q(4)=0.0f;
        q(5)=M_PI_2;
        q(6)=M_PI_2;
    }
    else
    {
        //this->q.setZero();
        //        q(0)=0.0f;
        //        q(1)=0.0f;
        //        q(2)=-M_PI_2;
        //        q(3)=0.0f;
        //        q(4)=M_PI_2;
        //        q(5)=-M_PI_2;
        q(0)=0.0f;
        q(1)=0.0f;
        q(2)=0.0f;
        q(3)=0.0f;
        q(4)=0.0f;
        q(5)=0.0f;
    }

#endif

    this->controller->setJointPosition(this->q);
    runThread=true;

}

RobotControl::~RobotControl()
{
    std::cout << " Robot is close *************************************************\n";
    this->controller->stop();
    this->controller->close();

    runThread=false;
}

void RobotControl::run()
{
    std::cout << " Robot is going to home position*************************************************\n";

#ifdef COACH
    moveToHome();
#endif

    std::thread thread_comau(&RobotControl::sendJointVectorComau, this);
    std::thread thread_coach(&RobotControl::sendJointVectorCoachObject, this);

    thread_coach.join();
    thread_comau.join();
    /********************  Main loop ************************************************/

}


void RobotControl::setRobotConfig(Eigen::VectorXd q)
{
    robotConfigMutex.lock();
    // std::cout << "set config:" << q.transpose() << std::endl;
    this->q = q;
    robotConfigMutex.unlock();
}

Eigen::VectorXd RobotControl::getRobotConfig()
{
    Eigen::VectorXd q_curr;
    robotConfigMutex.lock();
    q_curr = this->q;
    robotConfigMutex.unlock();
    return q_curr;
}

void
RobotControl::sendJointVectorCoachObject()
{
    rl::math::Vector q_iter;
    q_iter.resize(this->kinematic->getDof());
    std::string coach_cmd = "";
    while (this->runThread)
    {
        //update the robot state
        q_iter = this->getRobotConfig();        
        getCoachCommand(coach_cmd);
        /*************************************************************
         * Send robot Joints to coach
         *************************************************************/
    } // end while
}

void
RobotControl::sendJointVectorComau()
{
    rl::math::Vector q_iter;
    q_iter.resize(this->kinematic->getDof());

    while (this->runThread)
    {

        //update the robot state
        q_iter = this->getRobotConfig();
        this->controller->setJointPosition(q_iter);
        this->controller->step();

    } // end while
}

void RobotControl::setCoachCommand(std::string cmd)
{
    this->coachCmdMutex.lock();
    this->coach_cmd = cmd;
    this->coachCmdMutex.unlock();
}

void RobotControl::getCoachCommand(std::string &cmd)
{
    this->coachCmdMutex.lock();
    cmd = this->coach_cmd;
    this->coachCmdMutex.unlock();
}
        
void
RobotControl::move(rl::math::Vector& q, std::string coach_cmd, bool dummy)
{
    if(!dummy)
    {
        this->setRobotConfig(q);

#ifdef COACH
        this->controller->setJointPosition(q);
        this->controller->step();
#endif
    }    
    this->kinematic->setPosition(q);
    this->kinematic->forwardPosition();    
}

void
RobotControl::move(rl::math::Vector& q, bool dummy)
{

    //        for (std::size_t i = 0; i <= std::ceil(te / this->controller->getUpdateRate()); ++i)
    //        {
    //            for (std::size_t j = 0; j < this->controller->getDof(); ++j)
    //            {
    //                this->q(j) = interpolator[j].x(i * this->controller->getUpdateRate());
    //            }

    //            this->controller->setJointPosition(this->q);
    //            this->controller->step();
    //        }
//#ifdef COACH
//        this->setRobotConfig(q);
//#endif
    this->setRobotConfig(q);
    if(!dummy)
    {
#ifdef COACH
        this->controller->setJointPosition(q);
        this->controller->step();
#endif
    }    
    this->kinematic->setPosition(q);
    this->kinematic->forwardPosition();    
}

bool RobotControl::moveConfiguration(rl::math::Vector& x)
{
    this->move(q);
}

bool RobotControl::moveJoint(rl::math::Vector& x, bool dummy)
{
    this->move(x, dummy);
}

bool RobotControl::moveJoint(rl::math::Vector& x, std::string coach_cmd, bool dummy)
{
    this->move(x, coach_cmd, dummy);
}
void RobotControl::setStopRobotStatus(bool& status)
{
    stopRobotMoveMutex.lock();
    stopRobotMove=status;
    stopRobotMoveMutex.unlock();
}

void RobotControl::getStopRobotStatus(bool& status)
{
    stopRobotMoveMutex.lock();
    status=stopRobotMove;
    stopRobotMoveMutex.unlock();
}

bool RobotControl::movePose(rl::math::Transform& x, bool dummy)
{

    rl::math::Transform x_robot;
    x_robot = x;
    kinematic->setPosition(this->q);
    rl::math::Vector q(kinematic->getDof()); // TODO stable start for inverse kinematics
    QString robot_name_id(robotSemanticName.c_str());

    if(robot_name_id.contains("lwr"))
    {
        //this->q.setZero();
        q(0)=0.0f;
        q(1)=0.0f;
        q(2)=(170.0/180)*M_PI;
        q(3)=M_PI_2;
        q(4)=0.0f;
        q(5)=M_PI_2;
        q(6)=M_PI_2;
    }
    else if(robot_name_id.contains("comau"))
    {
        q(0)=0.0f;
        q(1)=0.0f;
        q(2)=-M_PI_2;
        q(3)=0.0f;
        q(4)=M_PI_2;
        q(5)=-M_PI_2;
    }

    kinematic->setPosition(q);
    kinematic->forwardPosition();

    if (!kinematic->calculateInversePosition(x_robot, 0, 0.5))
    {
        std::cout << "Move failed" << std::endl;

        return false;
    }

    rl::math::Vector p(this->controller->getDof());

    kinematic->getPosition(p);
    kinematic->setPosition(p);
    kinematic->forwardPosition();
    rl::math::Transform t1 = kinematic->getOperationalPosition(0);

//    if(t1.distance(x_robot, 0.1)>0.1f){
    if(::rl::math::transform::distance(t1, x_robot, 0.1f)>0.1f){
        std::cout << "There is a difference between desired and calculated positon, MOVE FAILED" << std::endl;

        return false;
    }

    this->move(p, dummy);

    return true;
}

std::string RobotControl::getRobotServiceNameTag()
{
    return robotName;
}

void RobotControl::moveToHome()
{

    rl::math::Vector joints(controller->getDof());

    QString robot_name_id(robotSemanticName.c_str());

    if(robot_name_id.contains("kuka"))
    {
        joints[0]=0.0f;
        joints[1]=0.0f;
        joints[2]=0.0f;
        joints[3]=0.0f;
        joints[4]=M_PI_2;
        joints[5]=0.0f;
        std::cout << "set joints for kuka" << std::endl;
    }
    else if(robot_name_id.contains("comau"))
    {
        joints[0]=0.0f;
        joints[1]=0.0f;
        joints[2]=-M_PI_2;
        joints[3]=0.0f;
        joints[4]=M_PI_2;
        joints[5]=-M_PI_2;
        std::cout << "set joints for comau" << std::endl;
    }
    else if(robot_name_id.contains("guedel-portico-light"))
    {

        //this->q.setZero();
        joints[0]=0.0f;
        joints[1]=0.0f;
        joints[2]=0.0f;
        joints[3]=0.0f;
        joints[4]=0.0f;
        joints[5]=0.0f;
        std::cout << "set joints for guedel-portico-light" << std::endl;
    }
    else if(robot_name_id.contains("lwr"))
    {

        //this->q.setZero();
        joints[0]=0.0f;
        joints[1]=0.0f;
        joints[2]=(170.0/180)*M_PI;
        joints[3]=M_PI_2;
        joints[4]=0.0f;
        joints[5]=M_PI_2;
        joints[6]=M_PI_2;
        std::cout << "set joints for lwr" << std::endl;
    }
    else if(robot_name_id.contains("ur5"))
    {

        //this->q.setZero();
        joints[0]=0.0f;
        joints[1]=-M_PI_2;
        joints[2]=-M_PI_2;
        joints[3]=-M_PI_2;
        joints[4]=M_PI_2;
        joints[5]=0.0f;
        std::cout << "set joints for ur5" << std::endl;
    }
    else
    {
        std::cout << "robot name not found" << robotSemanticName << std::endl;
        throw;
    }

//    kinematic->getHomePosition(joints);

    moveJoint(joints);
}

void RobotControl::getEFPose(rl::math::Transform& ef_pose)
{

//    kinematic->setPosition(this->getRobotConfig());
//    kinematic->forwardPosition();

    ef_pose=kinematic->getOperationalPosition(0);
}
