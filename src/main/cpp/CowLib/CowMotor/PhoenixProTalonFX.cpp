#include "PhoenixProTalonFX.h"

namespace CowMotor
{
    PhoenixProTalonFX::PhoenixProTalonFX(int id, std::string bus)
    {
        m_Talon = new ctre::phoenixpro::hardware::TalonFX(id, std::move(bus));
        m_Setpoint          = 0;
        m_UseFOC            = true;
        m_OverrideBrakeMode = false;
        m_OutputDirection = 1;

        // I think this just zeroes out the config so we start fresh each time
        ApplyConfig(ctre::phoenixpro::configs::TalonFXConfiguration{});
    }

    PhoenixProTalonFX::~PhoenixProTalonFX()
    {
        delete m_Talon;
    }

    /**
     * motor control requests
     **/

    void PhoenixProTalonFX::Set(std::variant<PercentOutput,
                                              VoltageOutput,
                                              PositionPercentOutput,
                                              PositionVoltage,
                                              VelocityPercentOutput,
                                              VelocityVoltage,
                                              MotionMagicPercentOutput,
                                              MotionMagicVoltage> request)
    {
        auto &talon            = m_Talon;
        double *setpoint       = &m_Setpoint;
        bool useFOC            = m_UseFOC;
        bool overrideBrakeMode = m_OverrideBrakeMode;
        int outputDirection    = m_OutputDirection;

        visit(
            [talon, setpoint, useFOC, overrideBrakeMode, outputDirection](auto &&req)
            {
                req.MultiplySetpoint(outputDirection);
                talon->SetControl(
                    req.ToControlRequest().WithEnableFOC(useFOC).WithOverrideBrakeDurNeutral(overrideBrakeMode));
                *setpoint = req.GetSetpoint();
            },
            request);
    }

    void PhoenixProTalonFX::Set(std::variant<TorqueCurrentOutput,
                                              PositionTorqueCurrent,
                                              VelocityTorqueCurrent,
                                              MotionMagicTorqueCurrent> request)
    {
        auto &talon      = m_Talon;
        double *setpoint = &m_Setpoint;
        int outputDirection = m_OutputDirection;
        visit(
            [talon, setpoint, outputDirection](auto &&req)
            {
                req.MultiplySetpoint(outputDirection);
                talon->SetControl(req.ToControlRequest());
                *setpoint = req.GetSetpoint();
            },
            request);
    }

    void PhoenixProTalonFX::Set(Follower request)
    {
        m_Talon->SetControl(request.ToControlRequest());
        m_Setpoint = request.LeaderID;
    }

    /**
     * configuration
     **/

    void PhoenixProTalonFX::UseFOC(bool useFOC)
    {
        m_UseFOC = useFOC;
    }
    
    void PhoenixProTalonFX::OverrideBrakeMode(bool overrideBrakeMode)
    {
        m_OverrideBrakeMode = overrideBrakeMode;
    }
    
    void PhoenixProTalonFX::ApplyConfig(std::variant<ctre::phoenixpro::configs::TalonFXConfiguration,
                                                      ctre::phoenixpro::configs::Slot0Configs,
                                                      ctre::phoenixpro::configs::MotionMagicConfigs,
                                                      ctre::phoenixpro::configs::MotorOutputConfigs> config)
    {
        auto &configuator = m_Talon->GetConfigurator();

        visit(
            [&configuator](auto &&config)
            {   
                // i think we initially put this in a loop and it stalled the bot
                ctre::phoenix::StatusCode res;
                // do
                // {
                res = configuator.Apply(config);
                // } while (!res.IsOK());
            },
            config);
    }

    /**
     * getters
     **/

    double PhoenixProTalonFX::GetSetpoint()
    {
        return m_Setpoint;
    }

    double PhoenixProTalonFX::GetPosition()
    {
        return m_Talon->GetPosition().Refresh().GetValue().value();
    }
    
    double PhoenixProTalonFX::GetVelocity()
    {
        return m_Talon->GetVelocity().Refresh().GetValue().value();
    }

    double PhoenixProTalonFX::GetTemp()
    {
        return m_Talon->GetDeviceTemp().Refresh().GetValue().value();
    }

    double PhoenixProTalonFX::GetInverted()
    {
        return m_Talon->GetInverted();
    }
    
    double PhoenixProTalonFX::GetTorqueCurrent()
    {
        return m_Talon->GetTorqueCurrent().GetValue().value();
    }
    
    double PhoenixProTalonFX::GetRefreshTorqueCurrent()
    {
        return m_Talon->GetTorqueCurrent().Refresh().GetValue().value();
    }

    CowMotor::NeutralMode PhoenixProTalonFX::GetNeutralMode()
    {
        auto config = ctre::phoenixpro::configs::MotorOutputConfigs{};
        m_Talon->GetConfigurator().Refresh(config);

        switch (config.NeutralMode.value)
        {
        case ctre::phoenixpro::signals::NeutralModeValue::Coast :
            return COAST;
        case ctre::phoenixpro::signals::NeutralModeValue::Brake :
            return BRAKE;
        default :
            return COAST;
        }
    }

    /**
     * setters
     **/

    int PhoenixProTalonFX::SetSensorPosition(double turns)
    {
        return m_Talon->SetRotorPosition(units::turn_t{ turns });
    }

    void PhoenixProTalonFX::SetNeutralMode(CowMotor::NeutralMode mode)
    {
        auto config = ctre::phoenixpro::configs::MotorOutputConfigs{};
        m_Talon->GetConfigurator().Refresh(config);

        switch (mode)
        {
        case COAST :
            config.NeutralMode = ctre::phoenixpro::signals::NeutralModeValue::Coast;
            break;
        case BRAKE :
            config.NeutralMode = ctre::phoenixpro::signals::NeutralModeValue::Brake;
            break;
        default :
            break;
        }

        // not sure why this doesnt use apply config
        // auto res = m_Talon->GetConfigurator().Apply(config);  commented to remove warning
        m_Talon->GetConfigurator().Apply(config);
    }

    void PhoenixProTalonFX::SetPID(double p, double i, double d, double f)
    {
        auto config = ctre::phoenixpro::configs::Slot0Configs{};

        config.kP = p;
        config.kI = i;
        config.kD = d;
        config.kV = f;

        ApplyConfig(config);
    }

    void PhoenixProTalonFX::SetMotionMagic(double velocity, double acceleration)
    {
        auto config = ctre::phoenixpro::configs::MotionMagicConfigs{};

        config.MotionMagicCruiseVelocity = velocity;
        config.MotionMagicAcceleration   = acceleration;

        ApplyConfig(config);
    }
    
    void PhoenixProTalonFX::SetInverted(bool inverted)
    {
        m_Talon->SetInverted(inverted);
    }

    void PhoenixProTalonFX::SetReversed(bool reversed)
    {
        if (reversed)
        {
            m_OutputDirection = -1;
        }
        else
        {
            m_OutputDirection = 1;
        }
    }
}