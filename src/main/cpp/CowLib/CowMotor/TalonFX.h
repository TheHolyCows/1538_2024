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
        struct SynchronizedSignals
        {
            ctre::phoenix6::StatusSignal<units::angle::turn_t> *Position;
            ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t> *Velocity;
            ctre::phoenix6::StatusSignal<units::angular_acceleration::turns_per_second_squared_t> *Acceleration;
            ctre::phoenix6::StatusSignal<units::ampere_t> *Current;
        };

        struct UnsynchronizedSignals
        {
            ctre::phoenix6::StatusSignal<units::celsius_t> *Temperature;
        };

        ctre::phoenix6::hardware::TalonFX m_Talon;
        ctre::phoenix6::configs::TalonFXConfiguration m_Config;

        SynchronizedSignals m_SynchronizedSignals;
        UnsynchronizedSignals m_UnsynchronizedSignals;

        ctre::phoenix::StatusCode ApplyConfig(ctre::phoenix6::configs::TalonFXConfiguration config);

    public:
        TalonFX(int id, std::string bus);

        std::vector<ctre::phoenix6::BaseStatusSignal*> GetSynchronizedSignals();
        std::vector<ctre::phoenix6::BaseStatusSignal*> GetUnsynchronizedSignals();
        ctre::phoenix::StatusCode FuseCANCoder(int id, double rotorToSensorRatio);
        ctre::phoenix::StatusCode ConfigRemoteCANCoder(int id);
        ctre::phoenix::StatusCode ConfigContinuousWrap(bool enable);
        ctre::phoenix::StatusCode ConfigMotionMagic(double kv, double ka);
        

        Status ConfigNeutralMode(NeutralMode neutralMode) override;
        Status ConfigPositivePolarity(Direction positivePolarity) override;
        Status ConfigPID(double kp, double ki, double kd, double ks = 0, FeedForwardType ffType = FeedForwardType::COSINE) override;
        Status ConfigStatorCurrentLimit(double current) override;

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
        Status Set(Control::DynamicMotionMagicTorqueCurrent request) override;
        Status Set(Control::Follower request) override;

        double GetPosition() override;
        double GetVelocity() override;
        double GetAcceleration() override;
        double GetCurrent() override;
        double GetTemperature() override;

        Status SetEncoderPosition(double value) override;

        void GetLogData(double *temp, double *encoder_count, bool *inverted) override;
    };
}