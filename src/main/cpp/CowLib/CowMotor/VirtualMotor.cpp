#include "VirtualMotor.h"

namespace CowMotor
{
    VirtualMotor::VirtualMotor(int id,  std::string bus)
    {
        printf("constructing motor: %i on bus %s\n",id,bus.c_str());
        m_Setpoint          = 0;
        m_UseFOC            = true;
        m_OverrideBrakeMode = false;
        m_OutputDirection = 1;
    }

    /**
     * motor control requests
     **/

    void VirtualMotor::Set(std::variant<PercentOutput,
                                              VoltageOutput,
                                              PositionPercentOutput,
                                              PositionVoltage,
                                              VelocityPercentOutput,
                                              VelocityVoltage,
                                              MotionMagicPercentOutput,
                                              MotionMagicVoltage> request)
    {
        return;
    }

    void VirtualMotor::Set(std::variant<TorqueCurrentOutput,
                                              PositionTorqueCurrent,
                                              VelocityTorqueCurrent,
                                              MotionMagicTorqueCurrent> request)
    {
        return;
    }

    void VirtualMotor::Set(Follower request)
    {
        return;
    }

    /**
     * configuration
     **/

    void VirtualMotor::UseFOC(bool useFOC)
    {
        m_UseFOC = useFOC;
    }
    
    void VirtualMotor::OverrideBrakeMode(bool overrideBrakeMode)
    {
        m_OverrideBrakeMode = overrideBrakeMode;
    }
    
    void VirtualMotor::ApplyConfig(std::variant<ctre::phoenix6::configs::TalonFXConfiguration,
                                                      ctre::phoenix6::configs::Slot0Configs,
                                                      ctre::phoenix6::configs::MotionMagicConfigs,
                                                      ctre::phoenix6::configs::MotorOutputConfigs> config)
    {
        return;
    }

    /**
     * getters
     **/

    double VirtualMotor::GetPosition()
    {
        return m_Setpoint;
    }
    
    double VirtualMotor::GetVelocity()
    {
        return 0.0;
    }
    
    double VirtualMotor::GetTorqueCurrent()
    {
        return 0.0;
    }
    
    double VirtualMotor::GetRefreshTorqueCurrent()
    {
        return 0.0;
    }

    CowMotor::NeutralMode VirtualMotor::GetNeutralMode()
    {
        return COAST;
    }

    /**
     * setters
     **/

    int VirtualMotor::SetSensorPosition(double turns)
    {
        return 0.0;
    }

    void VirtualMotor::SetNeutralMode(CowMotor::NeutralMode mode)
    {
        return;
    }

    void VirtualMotor::SetPID(double p, double i, double d, double f)
    {
        return;
    }

    void VirtualMotor::SetMotionMagic(double velocity, double acceleration)
    {
        return;
    }
    
    void VirtualMotor::SetInverted(bool inverted)
    {
        return;
    }

    void VirtualMotor::SetReversed(bool reversed)
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