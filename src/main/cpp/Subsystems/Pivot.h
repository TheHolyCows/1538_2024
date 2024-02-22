//==================================================
// Copyright (C) 2024 Team 1538 / The Holy Cows
// Elevator.h
// author: jon-bassi
// created on: 2024-2-5
//==================================================

#pragma once

#include "../CowConstants.h"
#include "../CowLib/Conversions.h"
#include "../CowLib/CowMotor/TalonFX.h"
#include "../CowLib/CowCANCoder.h"

class Pivot
{
public:

    Pivot(const int motorId1, const int motorId2, const int encoderId, int encoderOffset);

    std::vector<ctre::phoenix6::BaseStatusSignal*> GetSynchronizedSignals();

    double GetAngle(void);
    double GetSetpoint(void);

    void SetAngle(double angle);

    void BrakeMode(bool brakeMode);
    
    void ResetConstants(void);

    void Handle(void);

private:
    double m_TargetAngle;

    std::unique_ptr<CowMotor::TalonFX> m_PivotMotor1;
    std::unique_ptr<CowMotor::TalonFX> m_PivotMotor2;

    std::unique_ptr<CowLib::CowCANCoder> m_Encoder;

    CowMotor::Control::MotionMagicPositionDutyCycle m_PivotPosRequest = { 0 };
    CowMotor::Control::Follower m_FollowerRequest = { 0 };

};