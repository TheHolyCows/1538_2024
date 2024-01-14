#include "Intake.h"

Intake::Intake(int motorID1, int motorID2) 
{

    m_Motor1  = new CowLib::CowMotorController(motorID1, CowMotor::PHOENIX_V6);
    m_Motor2  = new CowLib::CowMotorController(motorID2, CowMotor::PHOENIX_V6);
    m_Motor1->SetNeutralMode(CowMotor::COAST);
    m_Motor2->SetNeutralMode(CowMotor::COAST);

}

void Intake::Set(double percent)
{

    m_Motor1ControlRequest.PercentOut = percent;
    m_Motor1ControlRequest.PercentOut = percent;

}

void Intake::Handle()
{

    if(m_Motor1)
    {
        m_Motor1->Set(m_Motor1ControlRequest);
    }

    if(m_Motor2)
    {
        m_Motor2->Set(m_Motor2ControlRequest);
    }



}

void Intake::ResetConstants(double kp, double ki, double kd, double ff)
{
    m_Motor1->SetPID(kp, ki, kd, ff = 0.0);
    m_Motor2->SetPID(kp, ki, kd, ff = 0.0);
}

double Intake::GetVelocity()
{
    double velocity = (m_Motor1->GetVelocity() +  m_Motor2->GetVelocity()) / 2;
    return velocity;
}

double Intake::GetMeanCurrent()
{
    double mcurrent = (m_Motor1->GetTorqueCurrent() + m_Motor2->GetTorqueCurrent()) / 2;
    return mcurrent;
}

double Intake::GetTotalCurrent()
{
    double tcurrent = m_Motor1->GetTorqueCurrent() + m_Motor2->GetTorqueCurrent();
    return tcurrent;
}
   