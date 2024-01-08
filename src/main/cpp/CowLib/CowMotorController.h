//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
//==================================================

#ifndef __COWLIB_COWMOTORCONTROLLER_H__
#define __COWLIB_COWMOTORCONTROLLER_H__

#include "CowMotor/CowMotorUtils.h"
#include "CowMotor/GenericCowMotor.h"
#include "CowMotor/PhoenixV6TalonFX.h"
#include <ctre/phoenix6/controls/MotionMagicTorqueCurrentFOC.hpp>

#include <variant>

namespace CowLib
{
    class CowMotorController
    {
    private:
        ctre::phoenix6::hardware::TalonFX *m_Talon;
        double m_Setpoint;
        bool m_UseFOC;
        bool m_OverrideBrakeMode;

        CowMotor::GenericCowMotor *m_GenericMotor;
        void InitializeInternalMotor(int id, CowMotor::MotorType, std::string bus);

    public:
        enum NeutralMode
        {
            COAST,
            BRAKE
        };

        CowMotorController(int id, CowMotor::MotorType motorType, std::string bus = "cowbus");

        ~CowMotorController();

        /* control requests */
        void Set(std::variant<CowMotor::PercentOutput,
                              CowMotor::VoltageOutput,
                              CowMotor::PositionPercentOutput,
                              CowMotor::PositionVoltage,
                              CowMotor::VelocityPercentOutput,
                              CowMotor::VelocityVoltage,
                              CowMotor::MotionMagicPercentOutput,
                              CowMotor::MotionMagicVoltage> request);
        void Set(std::variant<CowMotor::TorqueCurrentOutput, 
                              CowMotor::PositionTorqueCurrent, 
                              CowMotor::VelocityTorqueCurrent, 
                              CowMotor::MotionMagicTorqueCurrent> request);
        void Set(CowMotor::Follower request);

        /* configuration */
        void UseFOC(bool useFOC);
        void OverrideBrakeMode(bool overrideBrakeMode);
        void ApplyConfig(std::variant<ctre::phoenix6::configs::TalonFXConfiguration,
                                      ctre::phoenix6::configs::Slot0Configs,
                                      ctre::phoenix6::configs::MotionMagicConfigs,
                                      ctre::phoenix6::configs::MotorOutputConfigs> config);

        /* getters */
        double GetSetpoint();
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

        // not necessary?
        // ctre::phoenix6::hardware::TalonFX *GetInternalTalon();

        /* logging */
        void GetPIDData(double *setpoint, double *procVar, double *P, double *I, double *D);
        void GetLogData(double *temp, double *encoderCt, bool *isInverted);
    };
} // namespace CowLib

#endif /* __COWLIB_COWMOTORCONTROLLER_H__ */
