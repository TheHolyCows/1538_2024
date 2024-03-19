#include "Pivot.h"

Pivot::Pivot(const int motorId1, const int motorId2, const int encoderId, double encoderOffset)
{
    m_PivotMotor1 = std::make_unique<CowMotor::TalonFX>(motorId1, "cowdrive");
    m_PivotMotor2 = std::make_unique<CowMotor::TalonFX>(motorId2, "cowdrive");

    m_PrevBrakeMode = true;
    m_PivotMotor1->ConfigNeutralMode(CowMotor::NeutralMode::BRAKE);
    m_PivotMotor2->ConfigNeutralMode(CowMotor::NeutralMode::BRAKE);

    m_PivotMotor1->ConfigPositivePolarity(CowMotor::Direction::CLOCKWISE);
    m_PivotMotor2->ConfigPositivePolarity(CowMotor::Direction::COUNTER_CLOCKWISE);

    m_Encoder = std::make_unique<CowLib::CowCANCoder>(encoderId, "cowdrive");
    m_Encoder->ConfigAbsoluteOffset(encoderOffset);

    SetAngle(CONSTANT("PIVOT_STARTING_ANGLE"));

    m_FollowerRequest.MasterID = motorId1;
    m_FollowerRequest.OpposeMasterDirection = true;

    m_PivotMotor1->ConfigFusedCANCoder(encoderId, CONSTANT("PIVOT_GEAR_RATIO"));

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

    if (angle / 360.0 > m_PivotPosRequest.Position)
    {
        // Moving up
        m_PivotPosRequest.Velocity = CONSTANT("PIVOT_UP_V");
        m_PivotPosRequest.Acceleration = CONSTANT("PIVOT_UP_A");
        m_PivotPosRequest.Jerk = CONSTANT("PIVOT_UP_J");
    }
    else if (angle / 360.0 < m_PivotPosRequest.Position)
    {
        // Moving down
        m_PivotPosRequest.Velocity = CONSTANT("PIVOT_DOWN_V");
        m_PivotPosRequest.Acceleration = CONSTANT("PIVOT_DOWN_A");
        m_PivotPosRequest.Jerk = CONSTANT("PIVOT_DOWN_J");
    }

    m_PivotPosRequest.Position = angle / 360.0;
}

void Pivot::BrakeMode(bool brakeMode)
{
    if (brakeMode)
    {
        if (m_PivotMotor1)
        {
            if (m_PrevBrakeMode)
            {
                m_PivotMotor1->ConfigNeutralMode(CowMotor::BRAKE);
                m_PivotMotor2->ConfigNeutralMode(CowMotor::BRAKE);
                m_PrevBrakeMode = false;
            }
        }
    }
    else
    {
        if (m_PivotMotor1)
        {
            if (!m_PrevBrakeMode)
            {
                m_PivotMotor1->ConfigNeutralMode(CowMotor::COAST);
                m_PivotMotor2->ConfigNeutralMode(CowMotor::COAST);
                m_PrevBrakeMode = true;
            }
        }
    }
}

bool Pivot::AtTarget()
{
    double setpoint = GetSetpoint();
    double angle = GetAngle();

    if (std::abs(setpoint - angle) < 2.0)
    {
        return true;
    }
    else
    {
        return false;
    }

    // percent difference doesnt work well at higher values
    // double delta = std::abs(setpoint - angle);
    // double avg = (setpoint + angle) / 2.0;

    // double pctDiff = (delta / avg);

    // if (pctDiff < 0.03)
    // {
    //     return true;
    // }
    // else
    // {
    //     return false;
    // }
}

void Pivot::ResetConstants()
{
    m_PivotMotor1->ConfigPID(CONSTANT("PIVOT_P"),
                             CONSTANT("PIVOT_I"),
                             CONSTANT("PIVOT_D"),
                             CONSTANT("PIVOT_S"),
                             CONSTANT("PIVOT_V"),
                             CowMotor::FeedForwardType::COSINE);

    // m_PivotMotor1->ConfigMotionMagic(CONSTANT("PIVOT_V"),
    //                                  CONSTANT("PIVOT_A"));


    // m_PivotMotor2->ConfigPID(CONSTANT("PIVOT_P"),
    //                          CONSTANT("PIVOT_I"),
    //                          CONSTANT("PIVOT_D"));

    // m_PivotMotor2->ConfigMotionMagic(CONSTANT("PIVOT_V"),
    //                                  CONSTANT("PIVOT_A"));
}

void Pivot::Handle()
{
    double angleRad = (GetAngle() / 180) * 3.1415;
    m_PivotPosRequest.FeedForward = std::cos(angleRad) * CONSTANT("PIVOT_FF");

    if (m_PivotMotor1)
    {
        m_PivotMotor1->Set(m_PivotPosRequest);
    }
    if (m_PivotMotor2)
    {
        m_PivotMotor2->Set(m_FollowerRequest);
    }
}