//==================================================
// Copyright (C) 2024 Team 1538 / The Holy Cows
// @author dustinlieu
//==================================================

#include "TalonFX.h"

namespace CowMotor
{
    TalonFX::TalonFX(int id, std::string bus)
        : m_Talon(id, bus),
          m_SynchronizedSignals({
              .Position = &m_Talon.GetPosition(),
              .Velocity = &m_Talon.GetVelocity(),
              .Acceleration = &m_Talon.GetAcceleration(),
              .Current = &m_Talon.GetStatorCurrent()
          }),
          m_UnsynchronizedSignals({
              .Temperature = &m_Talon.GetDeviceTemp()
          })
    {
        m_Talon.GetConfigurator().Apply(m_Config);
        m_Talon.GetConfigurator().Refresh(m_Config);
    }

    ctre::phoenix::StatusCode TalonFX::ApplyConfig(ctre::phoenix6::configs::TalonFXConfiguration config)
    {
        ctre::phoenix::StatusCode status = m_Talon.GetConfigurator().Apply(config);

        if (!status.IsError())
        {
            m_Config = config;
        }

        return status;
    }

    std::vector<ctre::phoenix6::BaseStatusSignal*> TalonFX::GetSynchronizedSignals()
    {
        std::vector<ctre::phoenix6::BaseStatusSignal*> signals = {
            m_SynchronizedSignals.Position,
            m_SynchronizedSignals.Velocity,
            m_SynchronizedSignals.Acceleration,
            m_SynchronizedSignals.Current
        };

        return signals;
    }

    std::vector<ctre::phoenix6::BaseStatusSignal*> TalonFX::GetUnsynchronizedSignals()
    {
        std::vector<ctre::phoenix6::BaseStatusSignal*> signals = {
            m_UnsynchronizedSignals.Temperature
        };

        return signals;
    }

    ctre::phoenix::StatusCode TalonFX::FuseCANCoder(int id, double rotorToSensorRatio)
    {        
        ctre::phoenix6::configs::TalonFXConfiguration config = m_Config;
        config.Feedback.FeedbackRemoteSensorID = id;
        config.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;
        config.Feedback.SensorToMechanismRatio = 1.0;
        config.Feedback.RotorToSensorRatio = rotorToSensorRatio;

        return ApplyConfig(config);
    }

    ctre::phoenix::StatusCode TalonFX::ConfigRemoteCANCoder(int id)
    {
        ctre::phoenix6::configs::TalonFXConfiguration config = m_Config;
        config.Feedback.FeedbackRemoteSensorID = id;
        config.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
        config.Feedback.SensorToMechanismRatio = 1.0;

        return ApplyConfig(config);
    }

    ctre::phoenix::StatusCode TalonFX::ConfigContinuousWrap(bool enable)
    {
        ctre::phoenix6::configs::TalonFXConfiguration config = m_Config;
        config.ClosedLoopGeneral.ContinuousWrap = enable;

        return ApplyConfig(config);
    }

    ctre::phoenix::StatusCode TalonFX::ConfigMotionMagic(double kv, double ka)
    {
        ctre::phoenix6::configs::TalonFXConfiguration config = m_Config;
        config.MotionMagic.MotionMagicCruiseVelocity = kv;
        config.MotionMagic.MotionMagicAcceleration = ka;

        return ApplyConfig(config);
    }

    Status TalonFX::ConfigNeutralMode(NeutralMode neutralMode)
    {
        ctre::phoenix6::configs::TalonFXConfiguration config = m_Config;

        if (neutralMode == NeutralMode::COAST)
        {
            config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
        }
        else if (neutralMode == NeutralMode::BRAKE)
        {
            config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
        }

        return ApplyConfig(config);
    }

    Status TalonFX::ConfigPositivePolarity(Direction positivePolarity)
    {
        ctre::phoenix6::configs::TalonFXConfiguration config = m_Config;

        if (positivePolarity == Direction::CLOCKWISE)
        {
            config.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
        }
        else if (positivePolarity == Direction::COUNTER_CLOCKWISE)
        {
            config.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
        }

        return ApplyConfig(config);
    }

    Status TalonFX::ConfigPID(double kp, double ki, double kd, double kf)
    {
        ctre::phoenix6::configs::TalonFXConfiguration config = m_Config;
        config.Slot0.kP = kp;
        config.Slot0.kI = ki;
        config.Slot0.kD = kd;
        config.Slot0.kS = kf;

        return ApplyConfig(config);
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

    Status TalonFX::Set(Control::Follower cowRequest)
    {
        ctre::phoenix6::controls::Follower ctre_request = ctre::phoenix6::controls::Follower(cowRequest.MasterID, cowRequest.OpposeMasterDirection);

        return m_Talon.SetControl(ctre_request);
    }

    double TalonFX::GetPosition()
    {
        return m_SynchronizedSignals.Position->GetValue().value();
    }

    double TalonFX::GetVelocity()
    {
        return m_SynchronizedSignals.Velocity->GetValue().value();
    }

    double TalonFX::GetAcceleration()
    {
        return m_SynchronizedSignals.Acceleration->GetValue().value();
    }

    double TalonFX::GetCurrent()
    {
        return m_SynchronizedSignals.Current->GetValue().value();
    }

    double TalonFX::GetTemperature()
    {
        return m_UnsynchronizedSignals.Temperature->Refresh().GetValue().value();
    }

    Status TalonFX::SetEncoderPosition(double value)
    {
        return m_Talon.SetPosition(units::turn_t{ value });
    }

    void TalonFX::GetLogData(double *temp, double *encoder_count, bool *inverted)
    {
        *temp = GetTemperature();
        *encoder_count = GetPosition();
        *inverted = m_Config.MotorOutput.Inverted == ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
    }
}