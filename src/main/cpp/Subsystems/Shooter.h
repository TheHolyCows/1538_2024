//==================================================
// Copyright (C) 2024 Team 1538 / The Holy Cows
// Shooter.h
// author: Kiran/Dustin/Jon
// created on: 2024-1-16
//==================================================

#pragma once

#include <iostream>
#include "../CowLib/CowMotorController.h"
#include "../Cowconstants.h"

class Shooter
{
public:

    Shooter (int shooterID1, int shooterID2, int intakeID1, int intakeID2);
    void ResetConstants();
    void SetShooter (double percent);
    void SetIntake (double percent);
    double GetShooterVelocity();
    double GetIntakeVelocity();
    double GetMeanShooterCurrent();
    double GetTotalShooterCurrent();
    double GetMeanIntakeCurrent();
    double GetTotalIntakeCurrent();
    void Handle();
    
private:

    CowLib::CowMotorController *m_Shooter1;
    CowLib::CowMotorController *m_Shooter2;
    CowLib::CowMotorController *m_Intake1;
    CowLib::CowMotorController *m_Intake2;
    CowMotor::PercentOutput m_Shooter1ControlRequest{ 0 };
    CowMotor::PercentOutput m_Shooter2ControlRequest{ 0 };
    CowMotor::PercentOutput m_Intake1ControlRequest{ 0 };
    CowMotor::PercentOutput m_Intake2ControlRequest{ 0 };
};