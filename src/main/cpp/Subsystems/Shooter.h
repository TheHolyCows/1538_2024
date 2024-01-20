//==================================================
// Copyright (C) 2024 Team 1538 / The Holy Cows
// Shooter.h
// author: Kiran/Dustin/Jon
// created on: 2024-1-16
//==================================================

#pragma once

#include "../CowLib/CowMotorController.h"
#include "../Cowconstants.h"
#include "../Cowlib/CowLPF.h"
#include "../CowLib/Conversions.h"

class Shooter
{
public:

    Shooter (const int shooterID1, const int shooterID2, const int intakeID1, const int intakeID2, const int wristID);
    void ResetConstants();
    void SetShooter (double percent);
    void SetIntake (double percent);
    void SetWrist (double percent);
    double GetShooterVelocity();
    double GetIntakeVelocity();
    void RequestWristAngle(double position);
    double GetWristSetpoint();
    bool WristAtTarget();
    double GetWristAngle();
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
    CowLib::CowMotorController *m_Wrist;

    CowMotor::MotionMagicPercentOutput m_WristControlRequest{ 0 };

    CowMotor::PercentOutput m_ShooterControlRequest{ 0 };
    
    CowMotor::PercentOutput m_IntakeControlRequest{ 0 };
    
    double m_WristPosition;
};