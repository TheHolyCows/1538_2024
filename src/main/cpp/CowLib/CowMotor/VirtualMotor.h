//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
// @author ssemter, jon-bassi
//==================================================

#pragma once

#include "GenericCowMotor.h"
#include "CowMotorUtils.h"

#include <variant>


namespace CowMotor
{   
    /**
     * @brief virtual motor implementation for simultation testing
     */
    class VirtualMotor : public GenericCowMotor
    {
    public:
        VirtualMotor(int id, std::string bus);

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
        void ApplyConfig(std::variant<ctre::phoenix6::configs::TalonFXConfiguration,
                                              ctre::phoenix6::configs::Slot0Configs,
                                              ctre::phoenix6::configs::MotionMagicConfigs,
                                              ctre::phoenix6::configs::MotorOutputConfigs> config);

        /* getters */
        double GetPosition();
        double GetVelocity();
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
        void *m_Talon;
        
        double m_Setpoint;
        bool m_UseFOC;
        bool m_OverrideBrakeMode;
        int m_OutputDirection;
    };
}