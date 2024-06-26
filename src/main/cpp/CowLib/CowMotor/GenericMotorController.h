//==================================================
// Copyright (C) 2024 Team 1538 / The Holy Cows
// @author dustinlieu
//==================================================

#pragma once

#include <variant>

#include <ctre/phoenix/StatusCodes.h>

namespace CowMotor
{
    namespace Control
    {
        // Duty cycle controls
        struct DutyCycle
        {
            bool EnableFOC;

            // Units: Duty cycle between -1 and 1
            double DutyCycle;
        };

        struct PositionDutyCycle
        {
            bool EnableFOC;

            // Units: Motor turns
            double Position;

            // Units: Duty cycle between -1 and 1
            double FeedForward;
        };

        struct VelocityDutyCycle
        {
            bool EnableFOC;

            // Units: Motor turns per second
            double Velocity;

            // Units: Duty cycle between -1 and 1
            double FeedForward;
        };

        struct MotionMagicPositionDutyCycle
        {
            bool EnableFOC;

            // Units: Motor turns
            double Position;

            // Units: Duty cycle between -1 and 1
            double FeedForward;
        };

        struct MotionMagicVelocityDutyCycle
        {
            bool EnableFOC;

            // Units: Motor turns per second
            double Velocity;

            // Units: Duty cycle between -1 and 1
            double FeedForward;
        };

        // Torque current controls
        struct TorqueCurrent
        {
            // Units: Amperes
            double Current;

            // Units: Duty cycle between -1 and 1
            double MaxDutyCycle;

            // Units: Amperes
            double Deadband;
        };

        struct PositionTorqueCurrent
        {
            // Units: Motor turns
            double Position;

            // Units: Amperes
            double FeedForward;
        };

        struct VelocityTorqueCurrent
        {
            // Units: Motor turns per second
            double Velocity;

            // Units: Amperes
            double FeedForward;
        };

        struct MotionMagicPositionTorqueCurrent
        {
            // Units: Motor turns
            double Position;

            // Units: Amperes
            double FeedForward;
        };

        struct MotionMagicVelocityTorqueCurrent
        {
            // Units: Motor turns per second
            double Velocity;

            // Units: Amperes
            double FeedForward;
        };

        struct DynamicMotionMagicTorqueCurrent
        {
            // Units: Motor turns
            double Position;

            // Units: Motor turns / second
            double Velocity;

            // Units: Motor turns / second^2
            double Acceleration;

            // Units: Motor turns / second^3
            double Jerk;

            // Units: Amperes
            double FeedForward;

            // Slot number
            int Slot;
        };

        struct Follower
        {
            int MasterID;

            bool OpposeMasterDirection;
        };
    }

    typedef std::variant<ctre::phoenix::StatusCode> Status;

    enum NeutralMode
    {
        COAST,
        BRAKE
    };

    enum Direction
    {
        CLOCKWISE,
        COUNTER_CLOCKWISE
    };

    enum FeedForwardType
    {
        LINEAR,
        COSINE
    };

    class GenericMotorController
    {
    public:
        virtual Status ConfigNeutralMode(NeutralMode neutralMode) = 0;
        virtual Status ConfigPositivePolarity(Direction positivePolarity) = 0;
        virtual Status ConfigPID(double kp, double ki, double kd, double ks = 0, double kv = 0, double ka = 0, FeedForwardType ffType = FeedForwardType::LINEAR, int slot = 0) = 0;
        virtual Status ConfigStatorCurrentLimit(double current) = 0;

        virtual Status Set(Control::DutyCycle request) = 0;
        virtual Status Set(Control::PositionDutyCycle request) = 0;
        virtual Status Set(Control::VelocityDutyCycle request) = 0;
        virtual Status Set(Control::MotionMagicPositionDutyCycle request) = 0;
        virtual Status Set(Control::MotionMagicVelocityDutyCycle request) = 0;
        virtual Status Set(Control::TorqueCurrent request) = 0;
        virtual Status Set(Control::PositionTorqueCurrent request) = 0;
        virtual Status Set(Control::VelocityTorqueCurrent request) = 0;
        virtual Status Set(Control::MotionMagicPositionTorqueCurrent request) = 0;
        virtual Status Set(Control::MotionMagicVelocityTorqueCurrent request) = 0;
        virtual Status Set(Control::DynamicMotionMagicTorqueCurrent request) = 0;
        virtual Status Set(Control::Follower request) = 0;

        virtual double GetPosition() = 0;
        virtual double GetVelocity() = 0;
        virtual double GetAcceleration() = 0;
        virtual double GetTemperature() = 0;
        virtual double GetCurrent() = 0;

        virtual Status SetEncoderPosition(double value) = 0;

        virtual void GetLogData(double *temp, double *encoder_count, bool *inverted) = 0;
    };
}