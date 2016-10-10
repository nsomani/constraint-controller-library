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


#include "JoggingGUI.h"
#include "ui_JoggingGUI.h"

JoggingGUI::JoggingGUI(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::JoggingGUI)
{
    ui->setupUi(this);
}

JoggingGUI::~JoggingGUI()
{
    delete ui;
}

void JoggingGUI::on_pushButton_x_clicked()
{
    Eigen::VectorXd userOffsets(6);
    userOffsets << 0,0,0,0,0,0;
    userOffsets(0) = ui->doubleSpinBox_offset->value();
    if(ui->radioButton_moveRobot->isChecked())
        eval.stepDemoMoveRobot(userOffsets);
    else
        eval.stepDemoMoveObject(userOffsets);
}

void JoggingGUI::on_pushButton_y_clicked()
{
    Eigen::VectorXd userOffsets(6);
    userOffsets << 0,0,0,0,0,0;
    userOffsets(1) = ui->doubleSpinBox_offset->value();
    if(ui->radioButton_moveRobot->isChecked())
        eval.stepDemoMoveRobot(userOffsets);
    else
        eval.stepDemoMoveObject(userOffsets);
}

void JoggingGUI::on_pushButton_z_clicked()
{
    Eigen::VectorXd userOffsets(6);
    userOffsets << 0,0,0,0,0,0;
    userOffsets(2) = ui->doubleSpinBox_offset->value();
    if(ui->radioButton_moveRobot->isChecked())
        eval.stepDemoMoveRobot(userOffsets);
    else
        eval.stepDemoMoveObject(userOffsets);
}

void JoggingGUI::on_pushButton_a_clicked()
{
    Eigen::VectorXd userOffsets(6);
    userOffsets << 0,0,0,0,0,0;
    userOffsets(3) = ui->doubleSpinBox_offset->value();
    if(ui->radioButton_moveRobot->isChecked())
        eval.stepDemoMoveRobot(userOffsets);
    else
        eval.stepDemoMoveObject(userOffsets);
}

void JoggingGUI::on_pushButton_b_clicked()
{
    Eigen::VectorXd userOffsets(6);
    userOffsets << 0,0,0,0,0,0;
    userOffsets(4) = ui->doubleSpinBox_offset->value();
    if(ui->radioButton_moveRobot->isChecked())
        eval.stepDemoMoveRobot(userOffsets);
    else
        eval.stepDemoMoveObject(userOffsets);
}

void JoggingGUI::on_pushButton_c_clicked()
{
    Eigen::VectorXd userOffsets(6);
    userOffsets << 0,0,0,0,0,0;
    userOffsets(5) = ui->doubleSpinBox_offset->value();
    if(ui->radioButton_moveRobot->isChecked())
        eval.stepDemoMoveRobot(userOffsets);
    else
        eval.stepDemoMoveObject(userOffsets);
}

void JoggingGUI::setupDemo(RobotControl* &controller, std::string logger_fname, std::string solver_name, CompositePrioritySkill* &skill, bool offset_q, bool get_offset, bool plot)
{
    eval.setupDemo(controller, logger_fname, solver_name, skill, offset_q, get_offset, plot);
    ui->radioButton_moveObject->setVisible(false);
    ui->radioButton_moveRobot->setChecked(true);
}

void JoggingGUI::setupObstacleDemo(RobotControl* &controller, std::string logger_fname, std::string solver_name, CompositePrioritySkill* &skill, bool offset_q, bool get_offset, bool plot, rl::sg::Body *shape_body, int model_id, int body_id)
{
    eval.setupObstacleDemo(controller, logger_fname, solver_name, skill, offset_q, get_offset, plot, shape_body, model_id, body_id);
    ui->radioButton_moveObject->setVisible(true);
    ui->radioButton_moveRobot->setChecked(false);
}

void JoggingGUI::stepDemoMoveRobot(Eigen::VectorXd userOffsets)
{
    eval.stepDemoMoveRobot(userOffsets);
}

void JoggingGUI::stepDemoMoveObject(Eigen::VectorXd userOffsets)
{
    eval.stepDemoMoveObject(userOffsets);
}

void JoggingGUI::setMoveObject()
{
    ui->radioButton_moveObject->setChecked(true);
    ui->radioButton_moveRobot->setChecked(false);
}

void JoggingGUI::setMoveRobot()
{
    ui->radioButton_moveObject->setChecked(false);
    ui->radioButton_moveRobot->setChecked(true);
}

void JoggingGUI::setUserOffsets(double offset)
{
    ui->doubleSpinBox_offset->setValue(offset);
}

void JoggingGUI::on_checkBox_stateChanged(int arg1)
{
    eval.log = ui->checkBox->isChecked();
}
