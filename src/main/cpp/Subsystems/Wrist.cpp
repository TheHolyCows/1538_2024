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

    m_WristMotor->FuseCANCoder(encoderId, CONSTANT("WRIST_GEAR_RATIO"));

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
void Wrist::SetAngle(double angle, double pivotAngle)
{
    m_CanSetAngle = true;

    // compute angle of wrist relative to ground
    double angleSetpoint = angle + pivotAngle + 90;
    
    
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

    m_WristPosRequest.Position = angleSetpoint / 360.0;
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

void Wrist::ResetConstants()
{
    m_WristMotor->ConfigPID(CONSTANT("WRIST_P"),
                            CONSTANT("WRIST_I"),
                            CONSTANT("WRIST_D"));

    m_WristMotor->ConfigMotionMagic(CONSTANT("WRIST_V"),
                                    CONSTANT("WRIST_A"));

    m_CanSetAngle = true;
}

void Wrist::Handle()
{
    if (m_WristMotor)
    {
        m_WristMotor->Set(m_WristPosRequest);
    }
}