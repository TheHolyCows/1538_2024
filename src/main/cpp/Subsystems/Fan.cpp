#include "Fan.h" 

Fan::Fan(const int FanMotorID)
{
    m_FanMotor = std::make_unique<CowMotor::TalonFX>(FanMotorID, "cowbus");
    m_FanMotor->ConfigPositivePolarity(CowMotor::Direction::COUNTER_CLOCKWISE);
    m_FanMotor->ConfigNeutralMode(CowMotor::NeutralMode::BRAKE);

    m_FanState = FanState::IDLE;

    ResetConstants();
}

void Fan::ResetConstants()
{

}

Fan::FanState Fan::GetFanState()
{
    return m_FanState;
}

Fan::FanState Fan::UpdateFanState(FanState newstate)
{
    m_FanState = newstate;
}

double Fan::GetFanRPM()
{
    return m_FanMotor
}

