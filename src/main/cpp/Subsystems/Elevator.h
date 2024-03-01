//==================================================
// Copyright (C) 2024 Team 1538 / The Holy Cows
// Shooter.h
// author: dustinlieu
// created on: 2024-2-22
//==================================================

#pragma once

#include "Pivot.h"
#include "../CowLib/CowMotor/TalonFX.h"
#include "../Cowconstants.h"
#include "../CowLib/CowLogger.h"

class Elevator
{
public:

    Elevator(const int motorID1, const int motorID2);

    std::vector<ctre::phoenix6::BaseStatusSignal*> GetSynchronizedSignals();

    void ResetConstants();

    double GetPosition();
    double GetVelocity();
    double GetAcceleration();
    double GetCurrent();

    void SetExtension(double extensionLength);

    void Handle(Pivot *pivot);
    
private:
    std::unique_ptr<CowMotor::TalonFX> m_Motor1;
    std::unique_ptr<CowMotor::TalonFX> m_Motor2;

    CowMotor::Control::MotionMagicPositionDutyCycle m_PositionRequest;
    CowMotor::Control::Follower m_FollowerRequest;

    double m_TargetExtensionLength;
};