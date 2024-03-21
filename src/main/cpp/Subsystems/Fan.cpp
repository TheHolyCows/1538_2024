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

double Fan::GetFanVel()
{
    return m_FanMotor->GetVelocity();
}

void Fan::FanOn()
{
    m_FanState = FanState::IDLE;
}

void Fan::FanOff()
{
    m_FanState = FanState::SPIN_UP;
}

void Fan::Handle()

{
    switch (m_FanState)
    {
        case FanState::IDLE;
        {
            CowMotor::Control::TorqueCurrent request = {};
            m_FanMotor->Set(request);

            break;
        }

        case FanState::SPIN_UP:
        {
            CowMotor::Control::TorqueCurrent request = {};
            request.MaxDutyCycle = CONSTANT("FAN_MAX_DUTY_CYCLE");
            request.Current = CONSTANT("FAN_SPINUP_CURRENT");

            m_FanMotor->Set(request);

            if (m_FanMotor-> GetVelocity() > CONSTANT("FAN_VEL_THRESHOLD"))
            {
                m_FanState = FanState::FULL;
            }

            break;
        }

        case FanState::FULL:
        {
            // ??
        }
    
    }
}

