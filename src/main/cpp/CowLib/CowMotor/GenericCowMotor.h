//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
// @author jon-bassi
//==================================================

#pragma once

#include "CowMotorUtils.h"

#include <string>
#include <variant>
#include <ctre/phoenix6/TalonFX.hpp>

namespace CowMotor
{   
    /**
     * @brief template class for all motor controllers
     */
    class GenericCowMotor
    {
        public:
        GenericCowMotor();
        GenericCowMotor(int id, std::string bus);
        virtual ~GenericCowMotor();

        /* remainder of the class is pure virtual */


        /* control requests */
        virtual void Set(std::variant<CowMotor::PercentOutput,
                                      CowMotor::VoltageOutput,
                                      CowMotor::PositionPercentOutput,
                                      CowMotor::PositionVoltage,
                                      CowMotor::VelocityPercentOutput,
                                      CowMotor::VelocityVoltage,
                                      CowMotor::MotionMagicPercentOutput,
                                      CowMotor::MotionMagicVoltage> request) = 0;
        virtual void Set(std::variant<CowMotor::TorqueCurrentOutput, 
                                      CowMotor::PositionTorqueCurrent,
                                      CowMotor::VelocityTorqueCurrent, 
                                      CowMotor::MotionMagicTorqueCurrent> request) = 0;
        virtual void Set(CowMotor::Follower request) = 0;

        /* configuration */
        virtual void UseFOC(bool useFOC) = 0;
        virtual void OverrideBrakeMode(bool overrideBrakeMode) = 0;
        virtual void ApplyConfig(std::variant<ctre::phoenix6::configs::TalonFXConfiguration,
                                              ctre::phoenix6::configs::Slot0Configs,
                                              ctre::phoenix6::configs::MotionMagicConfigs,
                                              ctre::phoenix6::configs::MotorOutputConfigs> config) = 0;

        /* getters */
        virtual double GetSetpoint() = 0;
        virtual double GetPosition() = 0;
        virtual double GetVelocity() = 0;
        virtual double GetTemp() = 0;
        virtual double GetInverted() = 0;
        virtual double GetTorqueCurrent() = 0;
        virtual double GetRefreshTorqueCurrent() = 0;
        virtual CowMotor::NeutralMode GetNeutralMode() = 0;

        /* setters */
        virtual int SetSensorPosition(double turns) = 0;
        virtual void SetNeutralMode(CowMotor::NeutralMode mode) = 0;
        virtual void SetPID(double p, double i, double d, double f = 0.0) = 0;
        virtual void SetMotionMagic(double velocity, double acceleration) = 0;
        virtual void SetInverted(bool inverted) = 0;
        virtual void SetReversed(bool reversed) = 0;

        // ctre::phoenix6::hardware::TalonFX *GetInternalTalon();
    };
}