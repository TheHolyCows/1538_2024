//==================================================
// Copyright (C) 2024 Team 1538 / The Holy Cows
// @author dustinlieu
//==================================================

#include "TalonFX.h"

namespace CowMotor
{
    TalonFX::TalonFX(int id, std::string bus)
        : m_Talon(id, bus),
          m_MotorOutputConfig({}),
          m_SynchronizedStatusSignals({
              .Position = m_Talon.GetPosition(),
              .Velocity = m_Talon.GetVelocity(),
              .Acceleration = m_Talon.GetAcceleration()
          }),
          m_UnsynchronizedStatusSignals({
              .Temperature = m_Talon.GetDeviceTemp()
          })
    {
        ctre::phoenix6::configs::TalonFXConfigurator &configurator = m_Talon.GetConfigurator();
        configurator.Refresh(m_MotorOutputConfig);
    }

    std::vector<std::reference_wrapper<ctre::phoenix6::BaseStatusSignal>> TalonFX::GetSynchronizedStatusSignals()
    {
        std::vector<std::reference_wrapper<ctre::phoenix6::BaseStatusSignal>> statusSignals = {
            m_SynchronizedStatusSignals.Position,
            m_SynchronizedStatusSignals.Velocity,
            m_SynchronizedStatusSignals.Acceleration
        };

        return statusSignals;
    }

    std::vector<std::reference_wrapper<ctre::phoenix6::BaseStatusSignal>> TalonFX::GetUnsynchronizedStatusSignals()
    {
        std::vector<std::reference_wrapper<ctre::phoenix6::BaseStatusSignal>> statusSignals = {
            m_UnsynchronizedStatusSignals.Temperature
        };

        return statusSignals;
    }

    Status TalonFX::ConfigNeutralMode(NeutralMode neutralMode)
    {
        ctre::phoenix6::configs::TalonFXConfigurator &configurator = m_Talon.GetConfigurator();

        // Only update cached motor config if config was applied successfully
        ctre::phoenix6::configs::MotorOutputConfigs modifiedConfig = m_MotorOutputConfig;

        if (neutralMode == NeutralMode::COAST)
        {
            modifiedConfig.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
        }
        else if (neutralMode == NeutralMode::BRAKE)
        {
            modifiedConfig.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
        }

        ctre::phoenix::StatusCode status = configurator.Apply(modifiedConfig);

        if (!status.IsError())
        {
            m_MotorOutputConfig = modifiedConfig;
        }

        return status;
    }

    Status TalonFX::ConfigPositivePolarity(Direction positivePolarity)
    {
        ctre::phoenix6::configs::TalonFXConfigurator &configurator = m_Talon.GetConfigurator();

        // Only update cached motor config if config was applied successfully
        ctre::phoenix6::configs::MotorOutputConfigs modifiedConfig = m_MotorOutputConfig;

        if (positivePolarity == Direction::CLOCKWISE)
        {
            m_MotorOutputConfig.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
        }
        else if (positivePolarity == Direction::COUNTER_CLOCKWISE)
        {
            m_MotorOutputConfig.Inverted = ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
        }

        ctre::phoenix::StatusCode status = configurator.Apply(modifiedConfig);

        if (!status.IsError())
        {
            m_MotorOutputConfig = modifiedConfig;
        }

        return status;
    }

    Status TalonFX::ConfigPID(double kp, double ki, double kd, double kf)
    {
        ctre::phoenix6::configs::SlotConfigs slotConfigs = ctre::phoenix6::configs::SlotConfigs()
            .WithKP(kp)
            .WithKI(ki)
            .WithKD(kd);

        ctre::phoenix6::configs::TalonFXConfigurator &configurator = m_Talon.GetConfigurator();
        return configurator.Apply(slotConfigs);
    }

    Status TalonFX::Set(Control::DutyCycle cowRequest)
    {
        ctre::phoenix6::controls::DutyCycleOut ctre_request = ctre::phoenix6::controls::DutyCycleOut(units::scalar_t{ cowRequest.DutyCycle })
            .WithEnableFOC(cowRequest.EnableFOC);

        return m_Talon.SetControl(ctre_request);
    }

    Status TalonFX::Set(Control::PositionDutyCycle cowRequest)
    {
        ctre::phoenix6::controls::PositionDutyCycle ctre_request = ctre::phoenix6::controls::PositionDutyCycle(units::turn_t{ cowRequest.Position })
            .WithEnableFOC(cowRequest.EnableFOC)
            .WithFeedForward(cowRequest.FeedForward);

        return m_Talon.SetControl(ctre_request);
    }

    Status TalonFX::Set(Control::VelocityDutyCycle cowRequest)
    {
        ctre::phoenix6::controls::VelocityDutyCycle ctre_request = ctre::phoenix6::controls::VelocityDutyCycle(units::turns_per_second_t{ cowRequest.Velocity })
            .WithEnableFOC(cowRequest.EnableFOC)
            .WithFeedForward(cowRequest.FeedForward);

        return m_Talon.SetControl(ctre_request);
    }

    Status TalonFX::Set(Control::MotionMagicPositionDutyCycle cowRequest)
    {
        ctre::phoenix6::controls::MotionMagicDutyCycle ctre_request = ctre::phoenix6::controls::MotionMagicDutyCycle(units::turn_t{ cowRequest.Position })
            .WithEnableFOC(cowRequest.EnableFOC)
            .WithFeedForward(cowRequest.FeedForward);

        return m_Talon.SetControl(ctre_request);
    }

    Status TalonFX::Set(Control::MotionMagicVelocityDutyCycle cowRequest)
    {
        ctre::phoenix6::controls::MotionMagicVelocityDutyCycle ctre_request = ctre::phoenix6::controls::MotionMagicVelocityDutyCycle(units::turns_per_second_t{ cowRequest.Velocity })
            .WithEnableFOC(cowRequest.EnableFOC)
            .WithFeedForward(cowRequest.FeedForward);

        return m_Talon.SetControl(ctre_request);
    }

    Status TalonFX::Set(Control::TorqueCurrent cowRequest)
    {
        ctre::phoenix6::controls::TorqueCurrentFOC ctre_request = ctre::phoenix6::controls::TorqueCurrentFOC(units::ampere_t{ cowRequest.Current })
            .WithMaxAbsDutyCycle(units::scalar_t(cowRequest.MaxDutyCycle))
            .WithDeadband(units::ampere_t(cowRequest.Deadband));

        return m_Talon.SetControl(ctre_request);
    }

    Status TalonFX::Set(Control::PositionTorqueCurrent cowRequest)
    {
        ctre::phoenix6::controls::PositionTorqueCurrentFOC ctre_request = ctre::phoenix6::controls::PositionTorqueCurrentFOC(units::turn_t{ cowRequest.Position })
            .WithFeedForward(units::ampere_t{ cowRequest.FeedForward });

        return m_Talon.SetControl(ctre_request);
    }

    Status TalonFX::Set(Control::VelocityTorqueCurrent cowRequest)
    {
        ctre::phoenix6::controls::VelocityTorqueCurrentFOC ctre_request = ctre::phoenix6::controls::VelocityTorqueCurrentFOC(units::turns_per_second_t{ cowRequest.Velocity })
            .WithFeedForward(units::ampere_t{ cowRequest.FeedForward });

        return m_Talon.SetControl(ctre_request);
    }

    Status TalonFX::Set(Control::MotionMagicPositionTorqueCurrent cowRequest)
    {
        ctre::phoenix6::controls::MotionMagicTorqueCurrentFOC ctre_request = ctre::phoenix6::controls::MotionMagicTorqueCurrentFOC(units::turn_t{ cowRequest.Position })
            .WithFeedForward(units::ampere_t{ cowRequest.FeedForward });

        return m_Talon.SetControl(ctre_request);
    }

    Status TalonFX::Set(Control::MotionMagicVelocityTorqueCurrent cowRequest)
    {
        ctre::phoenix6::controls::MotionMagicVelocityTorqueCurrentFOC ctre_request = ctre::phoenix6::controls::MotionMagicVelocityTorqueCurrentFOC(units::turns_per_second_t{ cowRequest.Velocity })
            .WithFeedForward(units::ampere_t{ cowRequest.FeedForward });

        return m_Talon.SetControl(ctre_request);
    }

    double TalonFX::GetPosition()
    {
        return m_SynchronizedStatusSignals.Position.get().GetValue().value();
    }

    double TalonFX::GetVelocity()
    {
        return m_SynchronizedStatusSignals.Velocity.get().GetValue().value();
    }

    double TalonFX::GetAcceleration()
    {
        return m_SynchronizedStatusSignals.Acceleration.get().GetValue().value();
    }

    double TalonFX::GetTemperature()
    {
        ctre::phoenix6::StatusSignal<units::celsius_t> &statusSignal = m_UnsynchronizedStatusSignals.Temperature.get();
        statusSignal.Refresh();
        return statusSignal.GetValue().value();
    }

    Status TalonFX::SetEncoderPosition(double value)
    {
        return m_Talon.SetPosition(units::turn_t{ value });
    }

    void TalonFX::GetLogData(double *temp, double *encoder_count, bool *inverted)
    {
        *temp = GetTemperature();
        *encoder_count = GetPosition();
        *inverted = m_MotorOutputConfig.Inverted == ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
    }
}