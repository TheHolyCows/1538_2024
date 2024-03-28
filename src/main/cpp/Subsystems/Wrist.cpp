#include "Wrist.h"

Wrist::Wrist(const int motorId1, const int encoderId, double encoderOffset)
{
    m_WristMotor = std::make_unique<CowMotor::TalonFX>(motorId1, "cowbus");

    m_WristMotor->ConfigNeutralMode(CowMotor::NeutralMode::BRAKE);
    m_PrevBrakeMode = true;

    m_WristMotor->ConfigPositivePolarity(CowMotor::Direction::CLOCKWISE);

    m_Encoder = std::make_unique<CowLib::CowCANCoder>(encoderId, "cowbus");
    m_Encoder->ConfigAbsoluteOffset(encoderOffset);

    // SetAngle(CONSTANT("WRIST_STARTING_ANGLE"),CONSTANT("PIVOT_STARTING_ANGLE"));
    m_WristPosRequest.EnableFOC = true;

    m_WristMotor->ConfigFusedCANCoder(encoderId, CONSTANT("WRIST_GEAR_RATIO"));

    // set initial position to stop us breaking wrist
    SetAngle(CONSTANT("WRIST_GROUND_SETPOINT"), 0);

    ResetConstants();
}

std::vector<ctre::phoenix6::BaseStatusSignal*> Wrist::GetSynchronizedSignals()
{
    std::vector<ctre::phoenix6::BaseStatusSignal*> signals;
    std::vector<ctre::phoenix6::BaseStatusSignal*> wristSignals = m_WristMotor->GetSynchronizedSignals();

    signals.insert(signals.end(), wristSignals.begin(), wristSignals.end());

    return signals;
}

double Wrist::GetAngle()
{
    // return CowLib::Conversions::FalconToDegrees(m_WristMotor->GetPosition(), CONSTANT("WRIST_GEAR_RATIO"));
    return m_WristMotor->GetPosition() * 360.0;
}

double Wrist::GetSetpoint()
{
    //  return CowLib::Conversions::FalconToDegrees(m_WristPosRequest.Position, CONSTANT("WRIST_GEAR_RATIO"));
    return m_WristPosRequest.Position * 360.0;
}

/**
 * sets wrist to absolute angle relative to the ground
*/
void Wrist::SetAngle(double angle, double pivotSetpoint, bool force)
{
    m_WristState = WristState::DEFAULT;
    m_CanSetAngle = true;

    if (force)
    {
        m_TargetAngle = angle / 360.0;
        return;
    }

    // compute angle of wrist relative to ground
    // double angleSetpoint = angle + pivotSetpoint + 90;
    double angleSetpoint = 180 - (pivotSetpoint + angle) - CONSTANT("WRIST_VERT_OFFSET");

    if (angleSetpoint > CONSTANT("WRIST_MAX_ANGLE"))
    {
        angleSetpoint = CONSTANT("WRIST_MAX_ANGLE");
        m_CanSetAngle = false;
    }
    else if (angleSetpoint < CONSTANT("WRIST_MIN_ANGLE"))
    {
        angleSetpoint = CONSTANT("WRIST_MIN_ANGLE");
        m_CanSetAngle = false;
    }

    m_TargetAngle = (angleSetpoint / 360.0) + m_CurrentZeroOffset;
}

void Wrist::BrakeMode(bool brakeMode)
{
    if (brakeMode)
    {
        if (m_WristMotor)
        {
            if (m_PrevBrakeMode)
            {
                m_WristMotor->ConfigNeutralMode(CowMotor::BRAKE);
                m_PrevBrakeMode = false;
            }
        }
    }
    else
    {
        if (m_WristMotor)
        {
            if (!m_PrevBrakeMode)
            {
                m_WristMotor->ConfigNeutralMode(CowMotor::COAST);
                m_PrevBrakeMode = true;
            }
        }
    }
}

bool Wrist::AtTarget()
{
    return true;
}

void Wrist::ResetConstants()
{
    m_WristState = WristState::DEFAULT;
    
    m_WristMotor->ConfigPID(CONSTANT("WRIST_P"),
                            CONSTANT("WRIST_I"),
                            CONSTANT("WRIST_D"));

    m_WristMotor->ConfigMotionMagic(CONSTANT("WRIST_V"),
                                    CONSTANT("WRIST_A"),
                                    CONSTANT("WRIST_J"));

    m_CanSetAngle = true;
    m_CurrentZeroOffset = 0;
    m_ZeroFound = true;

    m_StowRequest.Current = CONSTANT("STOW_REQUEST_CURRENT");
    m_StowRequest.MaxDutyCycle = CONSTANT("STOW_REQUEST_MAX_DUTY_CYCLE");
}

void Wrist::Handle(Pivot *pivot)
{
    if (m_WristState == WristState::DEFAULT)
    {
        if (pivot->GetAngle() < CONSTANT("WRIST_SAFE_PIVOT_ANGLE"))
        {
            m_WristPosRequest.Position = std::max(CONSTANT("WRIST_SAFE_WRIST_ANGLE"), m_TargetAngle);
        }
        else
        {
            m_WristPosRequest.Position = m_TargetAngle;
        }

        m_WristMotor->Set(m_WristPosRequest);

    }
    else // stow
    {
        m_WristMotor->Set(m_StowRequest);

        if (m_ZeroFound)
        {
            m_ZeroTime = CowLib::GetTime();
            m_ZeroFound = false;
        }

        if (CowLib::GetTime() >= m_ZeroTime + 0.5)
        {
            if (m_WristMotor->GetVelocity() == 0.0)
            {
                m_ZeroFound = true;
                m_WristPosRequest.Position = m_WristMotor->GetPosition();
                m_CurrentZeroOffset = m_WristMotor->GetPosition() - CONSTANT("WRIST_ENCODER_MAX");
                m_WristState = WristState::DEFAULT;
            }
        }

    }
    // current checking for zero
    // if (GetAngle() >= (CONSTANT("WRIST_STOW_SETPOINT") - CONSTANT("WRIST_STOW_SETPOINT") * 0.1 ) && !m_ZeroFound)
    // {
    //     if (m_WristMotor->GetCurrent() > CONSTANT("WRIST_STALL_CURRENT"))
    //     {
    //         m_ZeroFound = true;
    //         m_CurrentZeroOffset = m_WristMotor->GetPosition() - CONSTANT("WRIST_ENCODER_MAX");
    //     }
    //     else
    //     {
    //         m_WristPosRequest.Position = m_WristPosRequest.Position + 0.015;
    //     }
    // }
}