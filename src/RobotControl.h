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

#ifndef _ROBOTCONTROL_H_
#define _ROBOTCONTROL_H_

#include <thread>
#include <time.h>
#include <boost/shared_ptr.hpp>
#include <rl/hal/Coach.h>
#include <rl/hal/Gnuplot.h>
#include <rl/math/Rotation.h>
#include <rl/mdl/Dynamic.h>
#include <rl/mdl/Model.h>
#include <rl/mdl/XmlFactory.h>
#include <rl/mdl/Body.h>
#include <QString>
#include <QThread>
#include <QMutex>
#include <QtNetwork>
#include <sstream>
#include <iostream>

#define COACH

class RobotControl : public QThread
{

public:
    RobotControl(const std::string& kinematics, const std::string& robot_name, const int& workcell_model_id, const std::string& coach_ip, int dof);

    virtual ~RobotControl();

    bool moveConfiguration(rl::math::Vector& x);

    bool moveJoint(rl::math::Vector& x, bool dummy=false);

    bool moveJoint(rl::math::Vector& x, std::string coach_cmd, bool dummy=false);

    bool movePose(rl::math::Transform& x, bool dummy=false);

    void moveToHome();

    void setStopRobotStatus(bool& status);

    void getStopRobotStatus(bool& status);

    void setRobotConfig(Eigen::VectorXd q);

    Eigen::VectorXd getRobotConfig();

    std::string getRobotServiceNameTag();

    void stopRobot();

    void getEFPose(rl::math::Transform& ef_pose);

    void sendJointVectorCoachObject();
    void sendJointVectorComau();

    void setCoachCommand(std::string cmd);
    void getCoachCommand(std::string &cmd);
    

    virtual void run();
    rl::math::Vector q;
    rl::math::Vector qp;
    rl::math::Vector qpp;

    std::string controller_ip;

    boost::shared_ptr< rl::mdl::Model > model;
#ifdef COACH
    rl::hal::Coach *controller;
#endif // COACH

    rl::mdl::Kinematic* kinematic;

private:
    bool runThread;

    std::string robotSemanticName;
    std::string robotName;

    void move(rl::math::Vector &q, std::string coach_cmd, bool dummy=false);
    void move(rl::math::Vector &q, bool dummy=false);

    QMutex robotConfigMutex;

    bool stopRobotMove;
    QMutex stopRobotMoveMutex;

    std::string coach_cmd;
    QMutex coachCmdMutex;

};

#endif // _ROBOTCONTROL_H_

