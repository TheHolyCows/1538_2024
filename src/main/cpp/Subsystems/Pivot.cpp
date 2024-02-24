#include "Pivot.h"

Pivot::Pivot(const int motorId1, const int motorId2, const int encoderId, int encoderOffset)
{
    m_PivotMotor1 = std::make_unique<CowMotor::TalonFX>(motorId1, "cowdrive");
    m_PivotMotor2 = std::make_unique<CowMotor::TalonFX>(motorId2, "cowdrive");

    m_PivotMotor1->ConfigNeutralMode(CowMotor::NeutralMode::BRAKE);
    m_PivotMotor2->ConfigNeutralMode(CowMotor::NeutralMode::BRAKE);

    m_PivotMotor1->ConfigPositivePolarity(CowMotor::Direction::CLOCKWISE);
    m_PivotMotor2->ConfigPositivePolarity(CowMotor::Direction::COUNTER_CLOCKWISE);

    m_Encoder = std::make_unique<CowLib::CowCANCoder>(encoderId, "cowdrive");
    m_Encoder->ConfigAbsoluteOffset(encoderOffset / 360.0);

    SetAngle(CONSTANT("PIVOT_STARTING_ANGLE"));
    m_PivotPosRequest.EnableFOC = true;

    m_FollowerRequest.MasterID = motorId1;
    m_FollowerRequest.OpposeMasterDirection = true;

    m_PivotMotor1->FuseCANCoder(encoderId, CONSTANT("PIVOT_GEAR_RATIO"));

    ResetConstants();
}

std::vector<ctre::phoenix6::BaseStatusSignal*> Pivot::GetSynchronizedSignals()
{
    std::vector<ctre::phoenix6::BaseStatusSignal*> signals;
    std::vector<ctre::phoenix6::BaseStatusSignal*> pivot1Signals = m_PivotMotor1->GetSynchronizedSignals();
    std::vector<ctre::phoenix6::BaseStatusSignal*> pivot2Signals = m_PivotMotor2->GetSynchronizedSignals();

    signals.insert(signals.end(), pivot1Signals.begin(), pivot1Signals.end());
    signals.insert(signals.end(), pivot2Signals.begin(), pivot2Signals.end());

    return signals;
}

double Pivot::GetAngle()
{
    // return CowLib::Conversions::FalconToDegrees(m_PivotMotor1->GetPosition(), CONSTANT("PIVOT_GEAR_RATIO"));
    return m_PivotMotor1->GetPosition() * 360.0;
}

double Pivot::GetSetpoint()
{
    //  return CowLib::Conversions::FalconToDegrees(m_PivotPosRequest.Position, CONSTANT("PIVOT_GEAR_RATIO"));
    return m_PivotPosRequest.Position * 360.0;
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

    m_PivotPosRequest.Position = angle / 360.0;
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

    m_PivotMotor1->ConfigMotionMagic(CONSTANT("PIVOT_V"),
                                     CONSTANT("PIVOT_A"));


    // m_PivotMotor2->ConfigPID(CONSTANT("PIVOT_P"),
    //                          CONSTANT("PIVOT_I"),
    //                          CONSTANT("PIVOT_D"));

    // m_PivotMotor2->ConfigMotionMagic(CONSTANT("PIVOT_V"),
    //                                  CONSTANT("PIVOT_A"));
}

void Pivot::Handle()
{
    if (m_PivotMotor1)
    {
        m_PivotMotor1->Set(m_PivotPosRequest);
    }
    if (m_PivotMotor2)
    {
        m_PivotMotor2->Set(m_FollowerRequest);
    }
}