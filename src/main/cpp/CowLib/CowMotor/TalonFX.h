//==================================================
// Copyright (C) 2024 Team 1538 / The Holy Cows
// @author dustinlieu
//==================================================

#pragma once

#include "GenericMotorController.h"

#include <ctre/phoenix6/TalonFX.hpp>

namespace CowMotor
{
    class TalonFX : public GenericMotorController
    {
    private:
        struct SynchronizedStatusSignals
        {
            std::reference_wrapper<ctre::phoenix6::StatusSignal<units::angle::turn_t>> Position;
            std::reference_wrapper<ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t>> Velocity;
            std::reference_wrapper<ctre::phoenix6::StatusSignal<units::angular_acceleration::turns_per_second_squared_t>> Acceleration;
        };

        struct UnsynchronizedStatusSignals
        {
            std::reference_wrapper<ctre::phoenix6::StatusSignal<units::celsius_t>> Temperature;
        };

        ctre::phoenix6::hardware::TalonFX m_Talon;
        ctre::phoenix6::configs::MotorOutputConfigs m_MotorOutputConfig;

        SynchronizedStatusSignals m_SynchronizedStatusSignals;
        UnsynchronizedStatusSignals m_UnsynchronizedStatusSignals;

    public:
        TalonFX(int id, std::string bus);

        std::vector<std::reference_wrapper<ctre::phoenix6::BaseStatusSignal>> GetSynchronizedStatusSignals();
        std::vector<std::reference_wrapper<ctre::phoenix6::BaseStatusSignal>> GetUnsynchronizedStatusSignals();

        Status ConfigNeutralMode(NeutralMode neutralMode) override;
        Status ConfigPositivePolarity(Direction positivePolarity) override;
        Status ConfigPID(double kp, double ki, double kd, double kf = 0) override;

        Status Set(Control::DutyCycle request) override;
        Status Set(Control::PositionDutyCycle request) override;
        Status Set(Control::VelocityDutyCycle request) override;
        Status Set(Control::MotionMagicPositionDutyCycle request) override;
        Status Set(Control::MotionMagicVelocityDutyCycle request) override;
        Status Set(Control::TorqueCurrent request) override;
        Status Set(Control::PositionTorqueCurrent request) override;
        Status Set(Control::VelocityTorqueCurrent request) override;
        Status Set(Control::MotionMagicPositionTorqueCurrent request) override;
        Status Set(Control::MotionMagicVelocityTorqueCurrent request) override;

        double GetPosition() override;
        double GetVelocity() override;
        double GetAcceleration() override;
        double GetTemperature() override;

        Status SetEncoderPosition(double value) override;

        void GetLogData(double *temp, double *encoder_count, bool *inverted) override;
    };
}