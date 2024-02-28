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

class Wrist
{
public:

    Wrist(const int motorId1, const int encoderId, double encoderOffset);

    std::vector<ctre::phoenix6::BaseStatusSignal*> GetSynchronizedSignals();

    double GetAngle(void);
    double GetSetpoint(void);

    void SetAngle(double angle, double pivotAngle);

    void BrakeMode(bool brakeMode);
    
    void ResetConstants(void);

    void Handle(void);

private:
    double m_TargetAngle;
    bool m_CanSetAngle;

    bool m_PrevBrakeMode;

    std::unique_ptr<CowMotor::TalonFX> m_WristMotor;

    std::unique_ptr<CowLib::CowCANCoder> m_Encoder;

    CowMotor::Control::MotionMagicPositionDutyCycle m_WristPosRequest = { };

};