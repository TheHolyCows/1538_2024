//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
// @author jon-bassi
//==================================================

#pragma once

#include <ctre/phoenix6/controls/DutyCycleOut.hpp>
#include <ctre/phoenix6/controls/VoltageOut.hpp>
#include <ctre/phoenix6/controls/TorqueCurrentFOC.hpp>
#include <ctre/phoenix6/controls/PositionDutyCycle.hpp>
#include <ctre/phoenix6/controls/PositionVoltage.hpp>
#include <ctre/phoenix6/controls/PositionTorqueCurrentFOC.hpp>
#include <ctre/phoenix6/controls/VelocityDutyCycle.hpp>
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>
#include <ctre/phoenix6/controls/VelocityTorqueCurrentFOC.hpp>
#include <ctre/phoenix6/controls/MotionMagicDutyCycle.hpp>
#include <ctre/phoenix6/controls/MotionMagicVoltage.hpp>
#include <ctre/phoenix6/controls/MotionMagicTorqueCurrentFOC.hpp>
#include <ctre/phoenix6/controls/Follower.hpp>

namespace CowMotor
{
    enum MotorType
    {
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
        // Percent of total motor output (-1 to 1)
        double PercentOut;

        void MultiplySetpoint(double multiplier);

        double GetSetpoint();

        ctre::phoenix6::controls::DutyCycleOut ToControlRequest();
    };

    struct VoltageOutput
    {
        // Voltage to set the motor to
        double Voltage;

        void MultiplySetpoint(double multiplier);

        double GetSetpoint();

        ctre::phoenix6::controls::VoltageOut ToControlRequest();
    };

    struct TorqueCurrentOutput
    {
        // Motor current in amps
        double Current;

        // Max absolute output of the motor controller (0 to 1)
        double MaxOutput = 1;

        // Deadband in amps. Deadband of 1 means the motor will stop quickly when set to 0
        double Deadband = 1;

        void MultiplySetpoint(double multiplier);

        double GetSetpoint();

        ctre::phoenix6::controls::TorqueCurrentFOC ToControlRequest();
    };

    struct PositionPercentOutput
    {
        // Position in turns
        double Position;

        // Feedforward in percent of total motor output (-1 to 1)
        double FeedForward = 0;

        void MultiplySetpoint(double multiplier);

        double GetSetpoint();

        ctre::phoenix6::controls::PositionDutyCycle ToControlRequest();
    };

    struct PositionVoltage
    {
        // Position in turns
        double Position;

        // Feedforward in volts
        double FeedForward = 0;

        void MultiplySetpoint(double multiplier);

        double GetSetpoint();

        ctre::phoenix6::controls::PositionVoltage ToControlRequest();
    };

    struct PositionTorqueCurrent
    {
        // Position in turns
        double Position;

        // Feedforward in amps
        double FeedForward = 0;

        void MultiplySetpoint(double multiplier);

        double GetSetpoint();

        ctre::phoenix6::controls::PositionTorqueCurrentFOC ToControlRequest();
    };

    struct VelocityPercentOutput
    {
        // Velocity in turns per second
        double Velocity;

        // Feedforward in percent of total motor output (-1 to 1)
        double FeedForward = 0;

        void MultiplySetpoint(double multiplier);

        double GetSetpoint();

        ctre::phoenix6::controls::VelocityDutyCycle ToControlRequest();
    };

    struct VelocityVoltage
    {
        // Velocity in turns per second
        double Velocity;

        // Feedforward in volts
        double FeedForward = 0;

        void MultiplySetpoint(double multiplier);

        double GetSetpoint();

        ctre::phoenix6::controls::VelocityVoltage ToControlRequest();
    };

    struct VelocityTorqueCurrent
    {
        // Velocity in turns per second
        double Velocity;

        // Feedforward in amps
        double FeedForward = 0;

        void MultiplySetpoint(double multiplier);

        double GetSetpoint();

        ctre::phoenix6::controls::VelocityTorqueCurrentFOC ToControlRequest();
    };

    struct MotionMagicPercentOutput
    {
        // Position in turns
        double Position;

        // Feedforward in percent of total motor output (-1 to 1)
        double FeedForward = 0;

        void MultiplySetpoint(double multiplier);

        double GetSetpoint();

        ctre::phoenix6::controls::MotionMagicDutyCycle ToControlRequest();
    };

    struct MotionMagicVoltage
    {
        // Position in turns
        double Position;

        // Feedforward in volts
        double FeedForward = 0;

        void MultiplySetpoint(double multiplier);

        double GetSetpoint();

        ctre::phoenix6::controls::MotionMagicVoltage ToControlRequest();
    };

    struct MotionMagicTorqueCurrent
    {
        // Position in turns
        double Position;

        // Feedforward in amps
        double FeedForward = 0;

        void MultiplySetpoint(double multiplier);

        double GetSetpoint();

        ctre::phoenix6::controls::MotionMagicTorqueCurrentFOC ToControlRequest();
    };

    struct Follower
    {
        // ID of the motor to follow
        int LeaderID;

        // Whether to invert the motor against the leader
        bool Invert = false;

        ctre::phoenix6::controls::Follower ToControlRequest() { return { LeaderID, Invert }; }
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