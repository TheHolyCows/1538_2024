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
#include "../CowLib/CowLogger.h"
#include "../Vision.h"

#include <cmath>

class Shooter
{
public:
    enum class IntakeState {
        IDLE,
        CALIBRATION_BEGIN,
        CALIBRATION_ACTIVE,
        CALIBRATION_END,
        DETECT_BEGIN,
        DETECT_ACTIVE,
        DETECT_HOLD,
        SHOOT,
        EXHAUST
    };

    enum class ShooterState {
        IDLE,
        SPIN_UP,
        READY
    };

    Shooter(const int shooterID1, const int shooterID2, const int intakeID, Vision* vision);

    std::vector<ctre::phoenix6::BaseStatusSignal*> GetSynchronizedSignals();

    void ResetConstants();

    IntakeState GetIntakeState();
    ShooterState GetShooterState();

    double GetIntakePosition();
    double GetIntakeVelocity();
    double GetIntakeAcceleration();
    double GetIntakeCurrent();

    double GetShooterVelocity();
    double GetShooterCurrent();

    bool IsReady();

    void StopIntake();
    void CalibrateIntake();
    void Intake();
    void Exhaust();

    void PrimeShooter();
    void StopShooter();

    void Shoot();

    void Handle();
    
private:
    std::unique_ptr<CowMotor::TalonFX> m_Shooter1;
    std::unique_ptr<CowMotor::TalonFX> m_Shooter2;
    std::unique_ptr<CowMotor::TalonFX> m_Intake;

    Vision *m_Vision;

    IntakeState m_IntakeState;
    ShooterState m_ShooterState;
 
    double m_IntakeCalibrationStartTime;
    double m_DetectStartTime;
    double m_IntakeGoalPosition;
 
    uint32_t m_CycleCount;
};