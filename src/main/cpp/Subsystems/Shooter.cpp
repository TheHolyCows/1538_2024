#include "Shooter.h"

Shooter::Shooter(const int shooterID1, const int shooterID2, const int intakeID1, const int intakeID2, const int wristID) 
{

    m_Shooter1 = std::make_unique<CowMotor::TalonFX>(shooterID1, "cowdrive");
    m_Shooter2 = std::make_unique<CowMotor::TalonFX>(shooterID2, "cowdrive");
    m_Shooter1->ConfigNeutralMode(CowMotor::NeutralMode::COAST);
    m_Shooter2->ConfigNeutralMode(CowMotor::NeutralMode::COAST);
    
    m_Intake1  = std::make_unique<CowMotor::TalonFX>(intakeID1, "cowdrive");
    m_Intake2  = std::make_unique<CowMotor::TalonFX>(intakeID2, "cowdrive");
    m_Intake1->ConfigNeutralMode(CowMotor::NeutralMode::COAST);
    m_Intake2->ConfigNeutralMode(CowMotor::NeutralMode::COAST);

    m_Wrist  = std::make_unique<CowMotor::TalonFX>(wristID, "cowdrive");
    m_Wrist->ConfigNeutralMode(CowMotor::NeutralMode::COAST);

    double m_WristPosition = 0;

    m_IntakeState = IntakeState::IDLE;
}

void Shooter::SetShooter(double percent)
{

    m_ShooterControlRequest.DutyCycle = percent;

}

void Shooter::SetIntake(double percent)
{

    m_IntakeControlRequest.DutyCycle = percent;

}

void Shooter::ResetConstants()
{
    m_Shooter1->ConfigPID(CONSTANT("SHOOTER_P"), CONSTANT("SHOOTER_I"), CONSTANT("SHOOTER_D"));
    m_Shooter2->ConfigPID(CONSTANT("SHOOTER_P"), CONSTANT("SHOOTER_I"), CONSTANT("SHOOTER_D"));
    m_Intake1->ConfigPID(CONSTANT("INTAKE_P"), CONSTANT("INTAKE_I"), CONSTANT("INTAKE_D"));
    m_Intake2->ConfigPID(CONSTANT("INTAKE_P"), CONSTANT("INTAKE_I"), CONSTANT("INTAKE_D"));
    m_Wrist->ConfigPID(CONSTANT("WRIST_P"), CONSTANT("WRIST_I"), CONSTANT("WRIST_D"), CONSTANT("WRIST_F"));
    m_Wrist->ConfigMotionMagic(CONSTANT("WRIST_V"), CONSTANT("WRIST_A"));
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
    // double mcurrent = (m_Shooter1->GetTorqueCurrent() + m_Shooter2->GetTorqueCurrent()) / 2;
    // return mcurrent;

    return 0;
}

double Shooter::GetIntakeCurrent()
{
    // double mcurrent = (m_Intake1->GetTorqueCurrent() + m_Intake2->GetTorqueCurrent()) / 2;
    // return mcurrent;

    return 0;
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

void Shooter::Preload()
{
    m_IntakeState = IntakeState::SPIN_UP;
}

void Shooter::Handle()
{
    if (m_IntakeState == IntakeState::IDLE)
    {
        CowMotor::Control::TorqueCurrent request;
        request.Current = 0;
        request.MaxDutyCycle = 0;
        request.Deadband = 0;

        m_Intake1->Set(request);
        m_Intake2->Set(request);
    }
    else if (m_IntakeState == IntakeState::SPIN_UP)
    {
        CowMotor::Control::TorqueCurrent request;
        request.Current = 5;
        request.MaxDutyCycle = 1;
        request.Deadband = 0;

        m_Intake1->Set(request);
        m_Intake2->Set(request);

        if (m_Intake1->GetVelocity() > 5)
        {
            m_IntakeState = IntakeState::WAIT;
        }
    }
    else if (m_IntakeState == IntakeState::WAIT)
    {

    }
    else if (m_IntakeState == IntakeState::MOVE)
    {
        
    }

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
   