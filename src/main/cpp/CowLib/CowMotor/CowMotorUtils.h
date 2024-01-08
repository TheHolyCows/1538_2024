//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
// @author jon-bassi
//==================================================

#pragma once

#include <ctre/phoenix/motorcontrol/ControlMode.h>
#include <ctre/phoenix6/TalonFX.hpp>

namespace CowMotor
{
    enum MotorType
    {
        PHOENIX_PRO,
        PHOENIX_V5,
        PHOENIX_V6,
        VIRTUAL = 0xFF
    };

    struct MotorConfiguration  // for creating drive motors/subsystems with multiple motors via a generic constructor is the idea, may need to go somewhere else
    {
        int id;
    };

    enum NeutralMode
    {
        COAST,
        BRAKE
    };


    /* motor control requests
       current valid requests for non-pro motor:
         - PercentOutput            : PercentOutput
         - PositionPercentOutput    : Postion
         - VelocityPercentOutput    : Velocity
         - MotionMagicPercentOutput : MotionMagic
    */
    struct PercentOutput
    {
        ctre::phoenix::motorcontrol::TalonFXControlMode GetControlMode() { return ctre::phoenix::motorcontrol::TalonFXControlMode::PercentOutput; };
        
        // Percent of total motor output (-1 to 1)
        double PercentOut;

        void MultiplySetpoint(double multiplier) { PercentOut = PercentOut * multiplier; };

        double GetSetpoint() { return PercentOut; };

        ctre::phoenix6::controls::DutyCycleOut ToControlRequest() { return { PercentOut }; };
    };

    struct VoltageOutput
    {
        ctre::phoenix::motorcontrol::TalonFXControlMode GetControlMode() { return ctre::phoenix::motorcontrol::TalonFXControlMode::Disabled; };
        
        // Voltage to set the motor to
        double Voltage;

        void MultiplySetpoint(double multiplier) { Voltage = Voltage * multiplier; };

        double GetSetpoint() { return Voltage; };

        ctre::phoenix6::controls::VoltageOut ToControlRequest() { return { units::volt_t{ Voltage } }; };
    };

    struct TorqueCurrentOutput
    {
        ctre::phoenix::motorcontrol::TalonFXControlMode GetControlMode() { return ctre::phoenix::motorcontrol::TalonFXControlMode::Disabled; };

        // Motor current in amps
        double Current;

        // Max absolute output of the motor controller (0 to 1)
        double MaxOutput = 1;

        // Deadband in amps. Deadband of 1 means the motor will stop quickly when set to 0
        double Deadband = 1;

        void MultiplySetpoint(double multiplier) { Current = Current * multiplier; };

        double GetSetpoint() { return Current; };

        ctre::phoenix6::controls::TorqueCurrentFOC ToControlRequest()
        {
            return { units::ampere_t{ Current }, MaxOutput, units::ampere_t{ Deadband }, false };
        }
    };

    struct PositionPercentOutput
    {
        ctre::phoenix::motorcontrol::TalonFXControlMode GetControlMode() { return ctre::phoenix::motorcontrol::TalonFXControlMode::Position; };

        // Position in turns
        double Position;

        // Feedforward in percent of total motor output (-1 to 1)
        double FeedForward = 0;

        void MultiplySetpoint(double multiplier) { Position = Position * multiplier; };

        double GetSetpoint() { return Position; };

        ctre::phoenix6::controls::PositionDutyCycle ToControlRequest()
        {
            return { units::turn_t{ Position }, true, FeedForward, 0, false };
        }
    };

    struct PositionVoltage
    {
        ctre::phoenix::motorcontrol::TalonFXControlMode GetControlMode() { return ctre::phoenix::motorcontrol::TalonFXControlMode::Disabled; };

        // Position in turns
        double Position;

        // Feedforward in volts
        double FeedForward = 0;

        void MultiplySetpoint(double multiplier) { Position = Position * multiplier; };

        double GetSetpoint() { return Position; };

        ctre::phoenix6::controls::PositionVoltage ToControlRequest()
        {
            return { units::turn_t{ Position }, true, units::volt_t{ FeedForward }, 0, false };
        };
    };

    struct PositionTorqueCurrent
    {
        ctre::phoenix::motorcontrol::TalonFXControlMode GetControlMode() { return ctre::phoenix::motorcontrol::TalonFXControlMode::Disabled; };

        // Position in turns
        double Position;

        // Feedforward in amps
        double FeedForward = 0;

        void MultiplySetpoint(double multiplier) { Position = Position * multiplier; };

        double GetSetpoint() { return Position; };

        ctre::phoenix6::controls::PositionTorqueCurrentFOC ToControlRequest()
        {
            return { units::turn_t{ Position }, units::ampere_t{ FeedForward }, 0, false };
        };
    };

    struct VelocityPercentOutput
    {
        ctre::phoenix::motorcontrol::TalonFXControlMode GetControlMode() { return ctre::phoenix::motorcontrol::TalonFXControlMode::Velocity; };

        // Velocity in turns per second
        double Velocity;

        // Feedforward in percent of total motor output (-1 to 1)
        double FeedForward = 0;

        void MultiplySetpoint(double multiplier) { Velocity = Velocity * multiplier; };

        double GetSetpoint() { return Velocity; };

        ctre::phoenix6::controls::VelocityDutyCycle ToControlRequest()
        {
            return { units::turns_per_second_t{ Velocity }, true, FeedForward, 0, false };
        };
    };

    struct VelocityVoltage
    {
        ctre::phoenix::motorcontrol::TalonFXControlMode GetControlMode() { return ctre::phoenix::motorcontrol::TalonFXControlMode::Disabled; };

        // Velocity in turns per second
        double Velocity;

        // Feedforward in volts
        double FeedForward = 0;

        void MultiplySetpoint(double multiplier) { Velocity = Velocity * multiplier; };

        double GetSetpoint() { return Velocity; };

        ctre::phoenix6::controls::VelocityVoltage ToControlRequest()
        {
            return { units::turns_per_second_t{ Velocity }, true, units::volt_t{ FeedForward }, 0, false };
        };
    };

    struct VelocityTorqueCurrent
    {
        ctre::phoenix::motorcontrol::TalonFXControlMode GetControlMode() { return ctre::phoenix::motorcontrol::TalonFXControlMode::Disabled; };

        // Velocity in turns per second
        double Velocity;

        // Feedforward in amps
        double FeedForward = 0;

        void MultiplySetpoint(double multiplier) { Velocity = Velocity * multiplier; };

        double GetSetpoint() { return Velocity; };

        ctre::phoenix6::controls::VelocityTorqueCurrentFOC ToControlRequest()
        {
            return { units::turns_per_second_t{ Velocity }, units::ampere_t{ FeedForward }, 0, false };
        };
    };

    struct MotionMagicPercentOutput
    {
        ctre::phoenix::motorcontrol::TalonFXControlMode GetControlMode() { return ctre::phoenix::motorcontrol::TalonFXControlMode::MotionMagic; };

        // Position in turns
        double Position;

        // Feedforward in percent of total motor output (-1 to 1)
        double FeedForward = 0;

        void MultiplySetpoint(double multiplier) { Position = Position * multiplier; };

        double GetSetpoint() { return Position; };

        ctre::phoenix6::controls::MotionMagicDutyCycle ToControlRequest()
        {
            return { units::turn_t{ Position }, true, FeedForward, 0, false };
        };
    };

    struct MotionMagicVoltage
    {
        ctre::phoenix::motorcontrol::TalonFXControlMode GetControlMode() { return ctre::phoenix::motorcontrol::TalonFXControlMode::Disabled; };

        // Position in turns
        double Position;

        // Feedforward in volts
        double FeedForward = 0;

        void MultiplySetpoint(double multiplier) { Position = Position * multiplier; };

        double GetSetpoint() { return Position; };

        ctre::phoenix6::controls::MotionMagicVoltage ToControlRequest()
        {
            return { units::turn_t{ Position }, true, units::volt_t{ FeedForward }, 0, false };
        };
    };

    struct MotionMagicTorqueCurrent
    {
        ctre::phoenix::motorcontrol::TalonFXControlMode GetControlMode() { return ctre::phoenix::motorcontrol::TalonFXControlMode::Disabled; };

        // Position in turns
        double Position;

        // Feedforward in amps
        double FeedForward = 0;

        void MultiplySetpoint(double multiplier) { Position = Position * multiplier; };

        double GetSetpoint() { return Position; };

        ctre::phoenix6::controls::MotionMagicTorqueCurrentFOC ToControlRequest()
        {
            return { units::turn_t{ Position }, FeedForward, 0, false };
        };
    };

    struct Follower
        {
            // ID of the motor to follow
            int LeaderID;

            // Whether to invert the motor against the leader
            bool Invert = false;

            ctre::phoenix6::controls::Follower ToControlRequest() { return { LeaderID, Invert }; };
        };


        /* configuration */
        /**
         * this is a pretty hack to be able to use config requests for Phoenix v5
         * without redefining them as structs here (like with the control requests)
         * the order of the enum must match the order of the variant specified in ApplyConfig()
         */
        enum ConfigRequestEn
        {
            TALON_FX_CFG = 0,
            SLOT_0_CFG,
            MOTION_MAGIC_CFG,
            MOTOR_OUT_CFG
        };
}