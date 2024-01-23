#include "Shooter.h"

Shooter::Shooter(const int shooterID1, const int shooterID2, const int intakeID1, const int intakeID2, const int wristID) 
{

    m_Shooter1  = new CowLib::CowMotorController(shooterID1, CowMotor::PHOENIX_V6);
    m_Shooter2  = new CowLib::CowMotorController(shooterID2, CowMotor::PHOENIX_V6);
    m_Shooter1->SetNeutralMode(CowMotor::COAST);
    m_Shooter2->SetNeutralMode(CowMotor::COAST);
    
    m_Intake1  = new CowLib::CowMotorController(intakeID1, CowMotor::PHOENIX_V6);
    m_Intake2  = new CowLib::CowMotorController(intakeID2, CowMotor::PHOENIX_V6);
    m_Intake1->SetNeutralMode(CowMotor::COAST);
    m_Intake2->SetNeutralMode(CowMotor::COAST);

    m_Wrist  = new CowLib::CowMotorController(wristID, CowMotor::PHOENIX_V6);
    m_Wrist->SetNeutralMode(CowMotor::BRAKE);

    double m_WristPosition = 0;
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
    m_Wrist->SetPID(CONSTANT("WRIST_P"), CONSTANT("WRIST_I"), CONSTANT("WRIST_D"), CONSTANT("WRIST_F"));
    m_Wrist->SetMotionMagic(CONSTANT("WRIST_V"), CONSTANT("WRIST_A"));
}

double Shooter::GetShooterVelocity()
{
    return m_Shooter1->GetVelocity();
}

double Shooter::GetIntakeVelocity()
{
    return m_Intake1->GetVelocity();
}

double Shooter::GetShooterCurrent()
{
    double mcurrent = (m_Shooter1->GetTorqueCurrent() + m_Shooter2->GetTorqueCurrent()) / 2;
    return mcurrent;
}

double Shooter::GetIntakeCurrent()
{
    double mcurrent = (m_Intake1->GetTorqueCurrent() + m_Intake2->GetTorqueCurrent()) / 2;
    return mcurrent;
}

void Shooter::RequestWristAngle(double angle)
{
    m_WristControlRequest.Position = CowLib::Conversions::DegreesToFalcon(angle, CONSTANT("WRIST_GEAR_RATIO")) * -1;
}

double Shooter::GetWristSetpoint()
{
    return CowLib::Conversions::FalconToDegrees(m_WristControlRequest.Position, CONSTANT("WRIST_GEAR_RATIO")) * -1;
}

bool Shooter::WristAtTarget()
{
    return fabs(GetWristSetpoint() - GetWristAngle() < CONSTANT("WRIST_TOLERANCE"));
}

double Shooter::GetWristAngle()
{
    return CowLib::Conversions::FalconToDegrees(m_Wrist->GetPosition(), CONSTANT("WRIST_GEAR_RATIO")) * -1;
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

    if (m_Wrist)
    {
        m_Wrist->Set(m_WristControlRequest);
    }
}
   