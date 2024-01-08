#include "CowMotorUtils.h"

namespace CowMotor
{
    // Percent output
    void PercentOutput::MultiplySetpoint(double multiplier)
    {
        PercentOut = PercentOut * multiplier;
    }

    double PercentOutput::GetSetpoint()
    {
        return PercentOut;
    }

    ctre::phoenix6::controls::DutyCycleOut PercentOutput::ToControlRequest()
    {
        return ctre::phoenix6::controls::DutyCycleOut(units::scalar_t{ PercentOut }, true);
    }

    // VoltageOutput
    void VoltageOutput::MultiplySetpoint(double multiplier)
    {
        Voltage = Voltage * multiplier;
    }

    double VoltageOutput::GetSetpoint() {
        return Voltage;
    };

    ctre::phoenix6::controls::VoltageOut VoltageOutput::ToControlRequest() {
        return ctre::phoenix6::controls::VoltageOut(units::volt_t{ Voltage }, true);
    }

    // TorqueCurrentOutput
    void TorqueCurrentOutput::MultiplySetpoint(double multiplier) { Current = Current * multiplier; }

    double TorqueCurrentOutput::GetSetpoint() { return Current; };

    ctre::phoenix6::controls::TorqueCurrentFOC TorqueCurrentOutput::ToControlRequest()
    {
        return ctre::phoenix6::controls::TorqueCurrentFOC(
            units::ampere_t{ Current },
            units::scalar_t { MaxOutput },
            units::ampere_t{ Deadband });
    }

    // PositionPercentOutput
    void PositionPercentOutput::MultiplySetpoint(double multiplier) { Position = Position * multiplier; }

    double PositionPercentOutput::GetSetpoint() { return Position; };

    ctre::phoenix6::controls::PositionDutyCycle PositionPercentOutput::ToControlRequest()
    {
        return ctre::phoenix6::controls::PositionDutyCycle(
            units::turn_t { Position },
            0_tps,
            true,
            units::scalar_t{ FeedForward });
    }

    // PositionVoltage
    void PositionVoltage::MultiplySetpoint(double multiplier) { Position = Position * multiplier; }

    double PositionVoltage::GetSetpoint() { return Position; };

    ctre::phoenix6::controls::PositionVoltage PositionVoltage::ToControlRequest()
    {
        return ctre::phoenix6::controls::PositionVoltage(
            units::turn_t{ Position },
            0_tps,
            true,
            units::volt_t{ FeedForward });
    }
    
    // PositionTorqueCurrent
    void PositionTorqueCurrent::MultiplySetpoint(double multiplier) { Position = Position * multiplier; }

    double PositionTorqueCurrent::GetSetpoint() { return Position; };

    ctre::phoenix6::controls::PositionTorqueCurrentFOC PositionTorqueCurrent::ToControlRequest()
    {
        return ctre::phoenix6::controls::PositionTorqueCurrentFOC(
            units::turn_t{ Position },
            0_tps,
            units::ampere_t{ FeedForward });
    }

    // VelocityPercentOutput
    void VelocityPercentOutput::MultiplySetpoint(double multiplier) { Velocity = Velocity * multiplier; }

    double VelocityPercentOutput::GetSetpoint() { return Velocity; };

    ctre::phoenix6::controls::VelocityDutyCycle VelocityPercentOutput::ToControlRequest()
    {
        return ctre::phoenix6::controls::VelocityDutyCycle(
            units::turns_per_second_t{ Velocity },
            0_tr_per_s_sq,
            true,
            units::scalar_t{ FeedForward });
    }

    // VelocityVoltage
    void VelocityVoltage::MultiplySetpoint(double multiplier) { Velocity = Velocity * multiplier; }

    double VelocityVoltage::GetSetpoint() { return Velocity; };

    ctre::phoenix6::controls::VelocityVoltage VelocityVoltage::ToControlRequest()
    {
        return ctre::phoenix6::controls::VelocityVoltage(
            units::turns_per_second_t{ Velocity },
            0_tr_per_s_sq,
            true,
            units::volt_t{ FeedForward });
    }

    // VelocityTorqueCurrent
    void VelocityTorqueCurrent::MultiplySetpoint(double multiplier) { Velocity = Velocity * multiplier; }

    double VelocityTorqueCurrent::GetSetpoint() { return Velocity; };

    ctre::phoenix6::controls::VelocityTorqueCurrentFOC VelocityTorqueCurrent::ToControlRequest()
    {
        return ctre::phoenix6::controls::VelocityTorqueCurrentFOC(
            units::turns_per_second_t{ Velocity },
            0_tr_per_s_sq,
            units::ampere_t{ FeedForward });
    }

    // MotionMagicPercentOutput
    void MotionMagicPercentOutput::MultiplySetpoint(double multiplier) { Position = Position * multiplier; }

    double MotionMagicPercentOutput::GetSetpoint() { return Position; };

    ctre::phoenix6::controls::MotionMagicDutyCycle MotionMagicPercentOutput::ToControlRequest()
    {
        return ctre::phoenix6::controls::MotionMagicDutyCycle(
            units::turn_t{ Position },
            true,
            units::scalar_t{ FeedForward });
    }

    // MotionMagicVoltage
    void MotionMagicVoltage::MultiplySetpoint(double multiplier) { Position = Position * multiplier; }

    double MotionMagicVoltage::GetSetpoint() { return Position; };

    ctre::phoenix6::controls::MotionMagicVoltage MotionMagicVoltage::ToControlRequest()
    {
        return ctre::phoenix6::controls::MotionMagicVoltage(
            units::turn_t{ Position },
            true,
            units::volt_t{ FeedForward });
    }

    // MotionMagicTorqueCurrent
    void MotionMagicTorqueCurrent::MultiplySetpoint(double multiplier) { Position = Position * multiplier; }

    double MotionMagicTorqueCurrent::GetSetpoint() { return Position; };

    ctre::phoenix6::controls::MotionMagicTorqueCurrentFOC MotionMagicTorqueCurrent::ToControlRequest()
    {
        return ctre::phoenix6::controls::MotionMagicTorqueCurrentFOC(
            units::turn_t{ Position },
            units::current::ampere_t{ FeedForward });
    }
}