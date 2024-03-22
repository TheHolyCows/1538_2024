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
#include "../CowLib/CowCANCoder.h"
#include "../Vision.h"

#include <cmath>

class Shooter
{
public:
    enum class IntakeState {
        IDLE,
        DETECT_ACTIVE,
        DETECT_HOLD,
        SHOOT,
        EXHAUST
    };

    enum class ShooterState {
        IDLE,
        SPIN_UP,
        READY,
        EXHAUST
    };

    Shooter(const int shooterID1, const int shooterID2, const int intakeID, const int cancoderID, Vision* vision);

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
    void Intake();
    void Exhaust();

    void PrimeShooter(double rps);
    void StopShooter();

    void Shoot(double intakeRPS);

    void UpdateIntakeState(IntakeState state, double intakeShootRPS);
    void UpdateShooterState(ShooterState state);

    void Handle();

private:
    std::unique_ptr<CowMotor::TalonFX> m_Shooter1;
    std::unique_ptr<CowMotor::TalonFX> m_Shooter2;
    std::unique_ptr<CowMotor::TalonFX> m_Intake;
    std::unique_ptr<CowLib::CowCANCoder> m_CANCoder;

    Vision *m_Vision;

    IntakeState m_IntakeState;
    ShooterState m_ShooterState;

    double m_IntakeGoalPosition;
    double m_IntakeShootRPS;
    double m_ShooterStartTime;
    double m_ShooterRPS;

    uint32_t m_CycleCount;
};