//==================================================
// Copyright (C) 2024 Team 1538 / The Holy Cows
// Intake.h
// author: Kiran/Dustin/Jon
// created on: 2024-1-16
//==================================================

#pragma once

#include <iostream>
#include "../CowLib/CowMotorController.h"

class Intake
{
public:

    Intake (int motorID1, int motorID2);
    void Handle();
    void Set (double percent);
    void ResetConstants(double kp, double ki, double kd, double ff);
    double GetVelocity();
    double GetMeanCurrent();
    double GetTotalCurrent();

private:

    CowLib::CowMotorController *m_Motor1;
    CowLib::CowMotorController *m_Motor2;
    CowMotor::PercentOutput m_Motor1ControlRequest{ 0 };
    CowMotor::PercentOutput m_Motor2ControlRequest{ 0 };

};