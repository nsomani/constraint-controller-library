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


#ifndef JOGGINGGUI_H
#define JOGGINGGUI_H

#include <QWidget>
#include "Evaluation.h"

namespace Ui {
class JoggingGUI;
}

class JoggingGUI : public QWidget
{
    Q_OBJECT

public:
    explicit JoggingGUI(QWidget *parent = 0);
    ~JoggingGUI();

    void setMoveObject();
    void setMoveRobot();
    void setUserOffsets(double offset);
    void setupDemo(RobotControl* &controller, std::string logger_fname, std::string solver_name, CompositePrioritySkill* &skill, bool offset_q, bool get_offset, bool plot=false);
    void setupObstacleDemo(RobotControl* &controller, std::string logger_fname, std::string solver_name, CompositePrioritySkill* &skill, bool offset_q, bool get_offset, bool plot, rl::sg::Body *shape_body, int model_id, int body_id);
    void stepDemoMoveRobot(Eigen::VectorXd userOffsets);
    void stepDemoMoveObject(Eigen::VectorXd userOffsets);
    Evaluation eval;

public slots:

    void on_pushButton_x_clicked();

    void on_pushButton_y_clicked();

    void on_pushButton_z_clicked();

    void on_pushButton_a_clicked();

    void on_pushButton_b_clicked();

    void on_pushButton_c_clicked();

    void on_checkBox_stateChanged(int arg1);

private:
    Ui::JoggingGUI *ui;


};

#endif // JOGGINGGUI_H
