//==================================================
// Copyright (C) 2024 Team 1538 / The Holy Cows
// Elevator.h
// author: jon-bassi
// created on: 2024-2-5
//==================================================

#pragma once

#include <cmath>
#include <optional>

#include "../CowConstants.h"
#include "../CowLib/Conversions.h"
#include "../CowLib/CowMotor/TalonFX.h"
#include "../CowLib/CowCANCoder.h"
#include "../CowLib/CowMotor/GenericMotorController.h"

class Pivot
{
public:
    Pivot(const int motorLeftID, const int motorRightID, const int encoderId, double encoderOffset);

    std::vector<ctre::phoenix6::BaseStatusSignal*> GetSynchronizedSignals();

    void ConfigNeutralMode(CowMotor::NeutralMode neutralMode);

    double GetAngle(void);
    double GetAngularVelocity();

    double GetTargetAngle(void);
    void SetTargetAngle(double angle);

    bool IsOnTarget(void);

    void ResetConstants(void);
    void Handle(double elevatorPos);

private:
    enum class State
    {
        IDLE,
        POSITION
    };

    struct MotorParameters
    {
        std::optional<double> offset;
        std::optional<double> backlash;
    };

    std::unique_ptr<CowMotor::TalonFX> m_MotorLeft;
    std::unique_ptr<CowMotor::TalonFX> m_MotorRight;
    std::unique_ptr<CowLib::CowCANCoder> m_Encoder;

    State m_State;

    MotorParameters m_LeftMotorParameters;
    MotorParameters m_RightMotorParameters;

    double m_TargetPosition;

    double GetLeftMotorPosition();
    double GetRightMotorPosition();

    double GetAbsoluteEncoderPosition();
    double GetAbsoluteEncoderVelocity();
};