#include "Shooter.h"

Shooter::Shooter(const int shooterID1, const int shooterID2, const int intakeID1, const int intakeID2) 
{

    m_Shooter1  = new CowLib::CowMotorController(shooterID1, CowMotor::PHOENIX_V6);
    m_Shooter2  = new CowLib::CowMotorController(shooterID2, CowMotor::PHOENIX_V6);
    m_Shooter1->SetNeutralMode(CowMotor::COAST);
    m_Shooter2->SetNeutralMode(CowMotor::COAST);
    
    m_Intake1  = new CowLib::CowMotorController(intakeID1, CowMotor::PHOENIX_V6);
    m_Intake2  = new CowLib::CowMotorController(intakeID2, CowMotor::PHOENIX_V6);
    m_Intake1->SetNeutralMode(CowMotor::COAST);
    m_Intake2->SetNeutralMode(CowMotor::COAST);

}

void Shooter::SetShooter(double percent)
{

    m_ShooterControlRequest.PercentOut = percent;

}

void Shooter::SetIntake(double percent)
{

    m_IntakeControlRequest.PercentOut = percent;

}

void Shooter::ResetConstants()
{
    m_Shooter1->SetPID(CONSTANT("SHOOTER_P"), CONSTANT("SHOOTER_I"), CONSTANT("SHOOTER_D"));
    m_Shooter2->SetPID(CONSTANT("SHOOTER_P"), CONSTANT("SHOOTER_I"), CONSTANT("SHOOTER_D"));
    m_Intake1->SetPID(CONSTANT("INTAKE_P"), CONSTANT("INTAKE_I"), CONSTANT("INTAKE_D"));
    m_Intake2->SetPID(CONSTANT("INTAKE_P"), CONSTANT("INTAKE_I"), CONSTANT("INTAKE_D"));
}

double Shooter::GetShooterVelocity()
{
    double velocity = (m_Shooter1->GetVelocity() +  m_Shooter2->GetVelocity()) / 2;
    return velocity;
}

double Shooter::GetIntakeVelocity()
{
    double velocity = (m_Intake1->GetVelocity() +  m_Intake2->GetVelocity()) / 2;
    return velocity;
}

double Shooter::GetMeanShooterCurrent()
{
    double mcurrent = (m_Shooter1->GetTorqueCurrent() + m_Shooter2->GetTorqueCurrent()) / 2;
    return mcurrent;
}

double Shooter::GetTotalShooterCurrent()
{
    double tcurrent = m_Shooter1->GetTorqueCurrent() + m_Shooter2->GetTorqueCurrent();
    return tcurrent;
}

double Shooter::GetMeanIntakeCurrent()
{
    double mcurrent = (m_Intake1->GetTorqueCurrent() + m_Intake2->GetTorqueCurrent()) / 2;
    return mcurrent;
}

double Shooter::GetTotalIntakeCurrent()
{
    double tcurrent = m_Intake1->GetTorqueCurrent() + m_Intake2->GetTorqueCurrent();
    return tcurrent;
}

void Shooter::Handle()
{

    if(m_Shooter1)
    {
        m_Shooter1->Set(m_ShooterControlRequest);
    }

    if(m_Shooter2)
    {
        m_Shooter2->Set(m_ShooterControlRequest);
    }

    if(m_Intake1)
    {
        m_Intake1->Set(m_IntakeControlRequest);
    }

    if(m_Intake2)
    {
        m_Intake2->Set(m_IntakeControlRequest);
    }

}
   