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

    class GenericMotorController
    {
    public:
        virtual Status ConfigNeutralMode(NeutralMode neutralMode) = 0;
        virtual Status ConfigPositivePolarity(Direction positivePolarity) = 0;
        virtual Status ConfigPID(double kp, double ki, double kd, double kf = 0) = 0;

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

        virtual double GetPosition() = 0;
        virtual double GetVelocity() = 0;
        virtual double GetAcceleration() = 0;
        virtual double GetTemperature() = 0;

        virtual Status SetEncoderPosition(double value) = 0;

        virtual void GetLogData(double *temp, double *encoder_count, bool *inverted) = 0;
    };
}