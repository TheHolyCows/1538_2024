#include "PhoenixV5TalonFX.h"

namespace CowMotor
{
    PhoenixV5TalonFX::PhoenixV5TalonFX(int id, std::string bus)
    {
        m_Talon = new ctre::phoenix::motorcontrol::can::TalonFX(id, std::move(bus));
        m_Setpoint = 0;
        m_OverrideBrakeMode = false;
        m_OutputDirection = 1;
        m_NeutralMode = CowMotor::BRAKE;
        SetNeutralMode(m_NeutralMode);
    }

    PhoenixV5TalonFX::~PhoenixV5TalonFX()
    {
        delete m_Talon;
    }

    /**
     * @brief from Talon code:
     * In PercentOutput, the output is between -1.0 and 1.0, with 0.0 as stopped.
     * In Current mode, output value is in amperes. (not setup currently)
     * In Velocity mode, output value is in rotations / 100ms.
     * In Position mode, output value is in rotations
     * In MotionMagic mode, output value is position of motor in rotations
     * In Follower mode, the output value is the integer device ID of the talon to
     *   duplicate. (below)
     * 
     * @param request 
     */
    void PhoenixV5TalonFX::Set(std::variant<PercentOutput,
                                              VoltageOutput,
                                              PositionPercentOutput,
                                              PositionVoltage,
                                              VelocityPercentOutput,
                                              VelocityVoltage,
                                              MotionMagicPercentOutput,
                                              MotionMagicVoltage> request)
    {
        auto &talon       = m_Talon;
        double *setpoint  = &m_Setpoint;
        int outputDirection = m_OutputDirection;
        auto &falcon_units_per_rot = FALCON_UNITS_PER_ROTATION;

        visit([talon, setpoint, outputDirection, falcon_units_per_rot](auto &&req)
        {
            ctre::phoenix::motorcontrol::TalonFXControlMode controlMode = req.GetControlMode();
            req.MultiplySetpoint(outputDirection);
            switch (controlMode)
            {
                case ctre::phoenix::motorcontrol::TalonFXControlMode::PercentOutput :
                    talon->Set(controlMode,req.GetSetpoint());
                    *setpoint = req.GetSetpoint();
                    break;
                case ctre::phoenix::motorcontrol::TalonFXControlMode::Position :
                    talon->Set(controlMode,req.GetSetpoint() * falcon_units_per_rot);
                    *setpoint = req.GetSetpoint() * falcon_units_per_rot;
                    break;
                case ctre::phoenix::motorcontrol::TalonFXControlMode::Velocity :
                    talon->Set(controlMode,req.GetSetpoint() * falcon_units_per_rot);
                    *setpoint = req.GetSetpoint() * falcon_units_per_rot;
                    break;
                case ctre::phoenix::motorcontrol::TalonFXControlMode::MotionMagic :
                    talon->Set(controlMode,req.GetSetpoint() * falcon_units_per_rot);
                    *setpoint = req.GetSetpoint() * falcon_units_per_rot;
                    break;
                case ctre::phoenix::motorcontrol::TalonFXControlMode::Disabled :
                    break;
                default:
                    break;
            }
        },
        request);
        
    }

    /**
     * @brief none of these are supported for TalonFX on Phoenix v5
     * 
     * @param request 
     */
    void PhoenixV5TalonFX::Set(std::variant<TorqueCurrentOutput,
                                              PositionTorqueCurrent,
                                              VelocityTorqueCurrent,
                                              MotionMagicTorqueCurrent> request)
    {
        return;
    }

    void PhoenixV5TalonFX::Set(Follower request)
    {
        m_Talon->Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Follower,request.LeaderID);
        m_Setpoint = request.LeaderID;
    }

    /**
     * configuration
     **/

    void PhoenixV5TalonFX::UseFOC(bool useFOC)
    {
        // no FOC on v5
        return;
    }
    
    void PhoenixV5TalonFX::OverrideBrakeMode(bool overrideBrakeMode)
    {
        // sets but does not do anything currently
        m_OverrideBrakeMode = overrideBrakeMode;
    }
    
    void PhoenixV5TalonFX::ApplyConfig(std::variant<ctre::phoenix6::configs::TalonFXConfiguration,
                                                      ctre::phoenix6::configs::Slot0Configs,
                                                      ctre::phoenix6::configs::MotionMagicConfigs,
                                                      ctre::phoenix6::configs::MotorOutputConfigs> config)
    {
        switch((int)config.index())
        {
            case CowMotor::ConfigRequestEn::TALON_FX_CFG :
                // nothing
                break;
            case CowMotor::ConfigRequestEn::SLOT_0_CFG :
                {
                    // closed loop slot 0 kP kI kD kV and kS
                    // same effect as using SetPID() but no F
                    ctre::phoenix6::configs::Slot0Configs slot0Cfg = std::get<ctre::phoenix6::configs::Slot0Configs>(config);
                    m_Talon->Config_kP(0, slot0Cfg.kP, 100);
                    m_Talon->Config_kI(0, slot0Cfg.kI, 100);
                    m_Talon->Config_kD(0, slot0Cfg.kD, 100);
                    break;
                }
            case CowMotor::ConfigRequestEn::MOTION_MAGIC_CFG :
                {
                    // motion magic accel and velocity
                    // config also contains Jerk which is unsued here
                    ctre::phoenix6::configs::MotionMagicConfigs mmCfg = std::get<ctre::phoenix6::configs::MotionMagicConfigs>(config);
                    m_Talon->ConfigMotionAcceleration(mmCfg.MotionMagicAcceleration, 100);
                    m_Talon->ConfigMotionCruiseVelocity(mmCfg.MotionMagicCruiseVelocity, 100);
                    break;
                }
            case CowMotor::ConfigRequestEn::MOTOR_OUT_CFG :
                // config for inverted, neutral mode, DutyCycleNeutralDeadband, PeakForwardDutyCycle, and PeakReverseDutyCycle
                // this is already covered in other commands, ignoring it for now
                break;
            default:
                break;
        }
    }

    /* getters */
    double PhoenixV5TalonFX::GetSetpoint()
    {
        return m_Setpoint;
    }
    
    double PhoenixV5TalonFX::GetPosition()
    {
        return m_Talon->GetSelectedSensorPosition() / FALCON_UNITS_PER_ROTATION;
    }
    
    double PhoenixV5TalonFX::GetVelocity()
    {
        return m_Talon->GetSelectedSensorVelocity() / FALCON_UNITS_PER_ROTATION;
    }

    double PhoenixV5TalonFX::GetTemp()
    {
        return m_Talon->GetTemperature();
    }

    double PhoenixV5TalonFX::GetInverted()
    {
        return m_Talon->GetInverted();
    }
    
    double PhoenixV5TalonFX::GetTorqueCurrent()
    {
        // not sure what this is in phoenix v5
        return 0.0;
    }
    
    double PhoenixV5TalonFX::GetRefreshTorqueCurrent()
    {
        // not sure what this is in phoenix v5
        return 0.0;
    }

    CowMotor::NeutralMode PhoenixV5TalonFX::GetNeutralMode()
    {
        return m_NeutralMode;
    }

    /* setters */
    int PhoenixV5TalonFX::SetSensorPosition(double turns)
    {
        return m_Talon->SetSelectedSensorPosition(0, turns*FALCON_UNITS_PER_ROTATION);
    }

    void PhoenixV5TalonFX::SetNeutralMode(CowMotor::NeutralMode mode)
    {
        m_NeutralMode = mode;
        switch (mode)
        {
            case CowMotor::BRAKE:
                m_Talon->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
                break;
            case CowMotor::COAST:
                m_Talon->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
                break;
            default:
                break;
        }
    }

    void PhoenixV5TalonFX::SetPID(double p, double i, double d, double f)
    {
        m_Talon->Config_kP(0, p, 0);
        m_Talon->Config_kI(0, i, 0);
        m_Talon->Config_kD(0, d, 0);
        m_Talon->Config_kF(0, f, 0);
    }

    void PhoenixV5TalonFX::SetMotionMagic(double velocity, double acceleration)
    {
        m_Talon->ConfigMotionAcceleration(acceleration,0);
        m_Talon->ConfigMotionCruiseVelocity(velocity,0);
    }

    void PhoenixV5TalonFX::SetInverted(bool inverted)
    {
        m_Talon->SetInverted(inverted);
    }
    
    void PhoenixV5TalonFX::SetReversed(bool reversed)
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