#include "CowMotorController.h"

#include "CowLogger.h"

#include <utility>

namespace CowLib
{
    /** 
     * @brief Construct a new Cow Motor Controller
     * @param id The CAN ID of the motor controller
     */
    CowMotorController::CowMotorController(int id, CowMotor::MotorType motorType, std::string bus)
    { 
        InitializeInternalMotor(id, motorType, bus);

        CowLogger::GetInstance()->RegisterMotor(id, this);
    }

    CowMotorController::~CowMotorController()
    {
        // delete m_Talon;
        delete m_GenericMotor;
    }

    void CowMotorController::InitializeInternalMotor(int id, CowMotor::MotorType motorType, std::string bus)
    {
        switch (motorType)
        {
            case CowMotor::PHOENIX_PRO:
                // m_GenericMotor = new CowMotor::PhoenixProTalonFX(id,bus);
                break;
            case CowMotor::PHOENIX_V5:
                m_GenericMotor = new CowMotor::PhoenixV5TalonFX(id,bus);
                break;
            case CowMotor::PHOENIX_V6:
                m_GenericMotor = new CowMotor::PhoenixV6TalonFX(id,bus);
                break;
            case CowMotor::VIRTUAL:
                break;
        }
    }

    /**
     * @brief Set the motor controller with a control request struct
     * @param request The control request struct
     */
    void CowMotorController::Set(std::variant<CowMotor::PercentOutput,
                                              CowMotor::VoltageOutput,
                                              CowMotor::PositionPercentOutput,
                                              CowMotor::PositionVoltage,
                                              CowMotor::VelocityPercentOutput,
                                              CowMotor::VelocityVoltage,
                                              CowMotor::MotionMagicPercentOutput,
                                              CowMotor::MotionMagicVoltage> request)
    {
        m_GenericMotor->Set(request);
    }

    /**
     * @brief Overload for TorqueControl requests because they always use FOC
     * 
     * @param request 
     */
    void CowMotorController::Set(std::variant<CowMotor::TorqueCurrentOutput,
                                              CowMotor::PositionTorqueCurrent,
                                              CowMotor::VelocityTorqueCurrent,
                                              CowMotor::MotionMagicTorqueCurrent> request)
    {
        m_GenericMotor->Set(request);
    }

    /**
     * @brief Overload for follwer request because it's special
     * 
     * @param request 
     */
    void CowMotorController::Set(CowMotor::Follower request)
    {
        m_GenericMotor->Set(request);
    }

    /** 
     * @brief Enables or disables the use of Field Oriented Control, default is true
     */
    void CowMotorController::UseFOC(bool useFOC)
    {
        m_GenericMotor->UseFOC(useFOC);
    }

    /**
     * @brief If overrided, seting an output of zero forces to motor to brake, default is false
     */
    void CowMotorController::OverrideBrakeMode(bool overrideBrakeMode)
    {
        m_GenericMotor->OverrideBrakeMode(overrideBrakeMode);
    }

    /**
     * @brief Applies a config to the motor controller
     * please ensure that any changes to the ordering of the variant is reflected in the ConfigRequestEn enum
     * in CowMotorUtils.h to ensure this function works with Phoenix v5 motors
     */
    void CowMotorController::ApplyConfig(std::variant<ctre::phoenix6::configs::TalonFXConfiguration,
                                                      ctre::phoenix6::configs::Slot0Configs,
                                                      ctre::phoenix6::configs::MotionMagicConfigs,
                                                      ctre::phoenix6::configs::MotorOutputConfigs> config)
    {
        m_GenericMotor->ApplyConfig(config);
    }

    /**
     * @brief Gets current setpoint of the motor
     * @return The current setpoint of the motor
    */
    double CowMotorController::GetSetpoint()
    {
        return m_GenericMotor->GetSetpoint();
    }

    /** 
     * @brief Gets the current position of the motor
     * @return The position in turns
     */
    double CowMotorController::GetPosition()
    {
        return m_GenericMotor->GetPosition();
    }

    /** 
     * @brief Gets the current velocity of the motor
     * @return The velocity in turns per second
     */
    double CowMotorController::GetVelocity()
    {
        return m_GenericMotor->GetVelocity();
    }

    /** 
     * @brief Gets the cached current corresponding to the torque output by the motor
     * this is split from the refresh current getter due to the possibility of flooding
     * the CANBUS or the motor with requests
     * @return The current in amps
     */
    double CowMotorController::GetTorqueCurrent()
    {
        return m_GenericMotor->GetTorqueCurrent();
    }

    /** 
     * @brief Gets the updated current corresponding to the torque output by the motor
     * this is split from the standard current getter due to the possibility of flooding
     * the CANBUS or the motor with requests
     * @return The current in amps
     */
    double CowMotorController::GetRefreshTorqueCurrent()
    {
        return m_GenericMotor->GetRefreshTorqueCurrent();
    }

    CowMotor::NeutralMode CowMotorController::GetNeutralMode()
    {
        return m_GenericMotor->GetNeutralMode();
    }

    /** 
     * @brief Sets the current position of the motor to a new value in turns. Used to zero.
     * @return Status code returned by talon
     */
    int CowMotorController::SetSensorPosition(double turns)
    {
        return m_GenericMotor->SetSensorPosition(turns);
    }

    void CowMotorController::SetNeutralMode(CowMotor::NeutralMode mode)
    {
        m_GenericMotor->SetNeutralMode(mode);
    }

    void CowMotorController::SetPID(double p, double i, double d, double f)
    {
        m_GenericMotor->SetPID(p,i,d,f);
    }

    void CowMotorController::SetMotionMagic(double velocity, double acceleration)
    {
        m_GenericMotor->SetMotionMagic(velocity,acceleration);
    }

    /**
     * @brief inverts the motor
     * honestly I dont think that this does what we think it does, use with caution
     * @param inverted true/false
     */
    void CowMotorController::SetInverted(bool inverted)
    {
        m_GenericMotor->SetInverted(inverted);
    }

    /**
     * @brief reverses the direction of the motor ouput by multiplying all set values by -1
     * not valid if using follower control requests
     * @param reversed true/false - true will set multiplier to -1, false (default) will be 1
     */
    void CowMotorController::SetReversed(bool reversed)
    {
        m_GenericMotor->SetReversed(reversed);
    }

    // ctre::phoenix6::hardware::TalonFX *CowMotorController::GetInternalTalon()
    // {
    //     return m_Talon;
    // }

    void CowMotorController::GetPIDData(double *setpoint, double *procVar, double *P, double *I, double *D)
    {
        *setpoint = m_GenericMotor->GetSetpoint();
        *procVar  = m_GenericMotor->GetPosition();
        *P        = -1;
        *I        = -1;
        *D        = -1;
    }

    void CowMotorController::GetLogData(double *temp, double *encoderCt, bool *isInverted)
    {
        *temp       = m_GenericMotor->GetTemp();
        *encoderCt  = m_GenericMotor->GetPosition();
        *isInverted = m_GenericMotor->GetInverted();
    }

} // namespace CowLib
