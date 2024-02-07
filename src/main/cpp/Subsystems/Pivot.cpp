#include "Pivot.h"

Pivot::Pivot(const int motorId1, const int motorId2, const int encoderId, int encoderOffset)
{
    m_PivotMotor1 = std::make_unique<CowMotor::TalonFX>(motorId1, "cowbus");
    m_PivotMotor2 = std::make_unique<CowMotor::TalonFX>(motorId2, "cowbus");

    m_PivotMotor2->ConfigNeutralMode(CowMotor::NeutralMode::BRAKE);
    m_PivotMotor2->ConfigNeutralMode(CowMotor::NeutralMode::BRAKE);

    m_PivotMotor1->ConfigPositivePolarity(CowMotor::Direction::COUNTER_CLOCKWISE);
    m_PivotMotor2->ConfigPositivePolarity(CowMotor::Direction::CLOCKWISE);

    m_Encoder = std::make_unique<CowLib::CowCANCoder>(encoderId, "cowbus");
    m_Encoder->ConfigAbsoluteOffset(encoderOffset / 360.0);

    m_TargetAngle = CONSTANT("PIVOT_STARTING_ANGLE");
    m_PivotMotor1->FuseCANCoder(encoderId, CONSTANT("PIVOT_TO_ENCODER_GEAR_RATIO"));

    ResetConstants();
}

double Pivot::GetAngle()
{
    return CowLib::Conversions::FalconToDegrees(m_PivotMotor1->GetPosition(), CONSTANT("PIVOT_GEAR_RATIO"));
}

double Pivot::GetSetpoint()
{
     return CowLib::Conversions::FalconToDegrees(m_PivotPosRequest.Position, CONSTANT("PIVOT_GEAR_RATIO"));
}

void Pivot::SetAngle(double angle)
{
    if (angle > CONSTANT("PIVOT_MAX_ANGLE"))
    {
        angle = CONSTANT("PIVOT_MAX_ANGLE");
    }
    else if (angle < CONSTANT("PIVOT_MIN_ANGLE"))
    {
        angle = CONSTANT("PIVOT_MIN_ANGLE");
    }

    m_PivotPosRequest.Position = CowLib::Conversions::DegreesToFalcon(angle, CONSTANT("PIVOT_GEAR_RATIO"));
}

void Pivot::BrakeMode(bool brakeMode)
{
    if (brakeMode)
    {
        if (m_PivotMotor1)
        {
            m_PivotMotor1->ConfigNeutralMode(CowMotor::BRAKE);
        }
        if (m_PivotMotor2)
        {
            m_PivotMotor2->ConfigNeutralMode(CowMotor::BRAKE);
        }
    }
    else
    {
        if (m_PivotMotor1)
        {
            m_PivotMotor1->ConfigNeutralMode(CowMotor::COAST);
        }
        if (m_PivotMotor2)
        {
            m_PivotMotor2->ConfigNeutralMode(CowMotor::COAST);
        }
    }
}

void Pivot::ResetConstants()
{
    m_PivotMotor1->ConfigPID(CONSTANT("PIVOT_P"),
                             CONSTANT("PIVOT_I"),
                             CONSTANT("PIVOT_D"));

    m_PivotMotor2->ConfigPID(CONSTANT("PIVOT_P"),
                             CONSTANT("PIVOT_I"),
                             CONSTANT("PIVOT_D"));

    m_PivotMotor1->ConfigMotionMagic(CONSTANT("PIVOT_V"),
                                     CONSTANT("PIVOT_A"));

    m_PivotMotor2->ConfigMotionMagic(CONSTANT("PIVOT_V"),
                                     CONSTANT("PIVOT_A"));
}

void Pivot::Handle()
{
    if (m_PivotMotor1)
    {
        m_PivotMotor1->Set(m_PivotPosRequest);
    }
    if (m_PivotMotor2)
    {
        m_PivotMotor2->Set(m_PivotPosRequest);
    }
    
}