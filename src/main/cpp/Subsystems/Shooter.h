//==================================================
// Copyright (C) 2024 Team 1538 / The Holy Cows
// Shooter.h
// author: Kiran/Dustin/Jon
// created on: 2024-1-16
//==================================================

#pragma once

#include "../CowLib/CowMotor/TalonFX.h"
#include "../Cowconstants.h"
#include "../Cowlib/CowLPF.h"
#include "../CowLib/Conversions.h"

class Shooter
{
public:

    Shooter (const int shooterID1, const int shooterID2, const int intakeID1, const int intakeID2, const int wristID);

    std::vector<ctre::phoenix6::BaseStatusSignal*> GetSynchronizedSignals();

    void ResetConstants();
    void SetShooter (double percent);
    void SetIntake (double percent);

    double GetShooterVelocity();
    
    double GetIntakeVelocity();

    void RequestWristAngle(double position);
    double GetWristSetpoint();
    bool WristAtTarget();
    double GetWristAngle();

    double GetShooterCurrent();
    double GetIntakeCurrent();

    void Preload();
    void PreloadStop();
    void Handle();
    
private:
    enum IntakeState {
        IDLE,
        SPIN_UP,
        WAIT,
        MOVE
    };

    std::unique_ptr<CowMotor::TalonFX> m_Shooter1;
    std::unique_ptr<CowMotor::TalonFX> m_Shooter2;
    std::unique_ptr<CowMotor::TalonFX> m_Intake1;
    std::unique_ptr<CowMotor::TalonFX> m_Intake2;
    std::unique_ptr<CowMotor::TalonFX> m_Wrist;

    CowMotor::Control::MotionMagicPositionDutyCycle m_WristControlRequest{ 0 };

    CowMotor::Control::DutyCycle m_ShooterControlRequest{ 0 };
    
    CowMotor::Control::DutyCycle m_IntakeControlRequest{ 0 };
    
    double m_WristPosition;
    double m_Intake1GoalPosition;
    double m_Intake2GoalPosition;
    IntakeState m_IntakeState;
};