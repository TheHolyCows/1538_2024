//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
// @author ssemtner, jon-bassi
//==================================================

#pragma once

#include "GenericCowMotor.h"
#include "CowMotorUtils.h"

#include <ctre/phoenixpro/TalonFX.hpp>
#include <variant>


namespace CowMotor
{
    class PhoenixProTalonFX : public GenericCowMotor
    {
    public:
        PhoenixProTalonFX(int id, std::string bus);
        ~PhoenixProTalonFX();

        /* control requests */
        void Set(std::variant<PercentOutput,
                              VoltageOutput,
                              PositionPercentOutput,
                              PositionVoltage,
                              VelocityPercentOutput,
                              VelocityVoltage,
                              MotionMagicPercentOutput,
                              MotionMagicVoltage> request);

        void Set(std::variant<TorqueCurrentOutput, 
                              PositionTorqueCurrent, 
                              VelocityTorqueCurrent, 
                              MotionMagicTorqueCurrent> request);

        void Set(Follower request);

        /* configuration */
        void UseFOC(bool useFOC);
        void OverrideBrakeMode(bool overrideBrakeMode);
        void ApplyConfig(std::variant<ctre::phoenixpro::configs::TalonFXConfiguration,
                                              ctre::phoenixpro::configs::Slot0Configs,
                                              ctre::phoenixpro::configs::MotionMagicConfigs,
                                              ctre::phoenixpro::configs::MotorOutputConfigs> config);

        /* getters */
        double GetSetpoint();
        double GetPosition();
        double GetVelocity();
        double GetTemp();
        double GetInverted();
        double GetTorqueCurrent();
        double GetRefreshTorqueCurrent();
        CowMotor::NeutralMode GetNeutralMode();

        /* setters */
        int SetSensorPosition(double turns);
        void SetNeutralMode(CowMotor::NeutralMode mode);
        void SetPID(double p, double i, double d, double f = 0.0);
        void SetMotionMagic(double velocity, double acceleration);
        void SetInverted(bool inverted);
        void SetReversed(bool reversed);

    private:
        ctre::phoenixpro::hardware::TalonFX *m_Talon;
        
        double m_Setpoint;
        bool m_UseFOC;
        bool m_OverrideBrakeMode;
        int m_OutputDirection;
    };
}