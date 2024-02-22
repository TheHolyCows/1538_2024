#include "Shooter.h"

Shooter::Shooter(const int shooterID1, const int shooterID2, const int intakeID)
{

    m_Shooter1 = std::make_unique<CowMotor::TalonFX>(shooterID1, "cowdrive");
    m_Shooter2 = std::make_unique<CowMotor::TalonFX>(shooterID2, "cowdrive");

    m_Shooter1->ConfigNeutralMode(CowMotor::NeutralMode::COAST);
    m_Shooter2->ConfigNeutralMode(CowMotor::NeutralMode::COAST);

    m_Shooter1->ConfigPositivePolarity(CowMotor::Direction::COUNTER_CLOCKWISE);
    m_Shooter2->ConfigPositivePolarity(CowMotor::Direction::CLOCKWISE);
    
    m_Intake = std::make_unique<CowMotor::TalonFX>(intakeID, "cowdrive");
    m_Intake->ConfigNeutralMode(CowMotor::NeutralMode::BRAKE);

    m_IntakeState = IntakeState::IDLE;
    m_ShooterState = ShooterState::IDLE;
}

std::vector<ctre::phoenix6::BaseStatusSignal*> Shooter::GetSynchronizedSignals()
{
    std::vector<ctre::phoenix6::BaseStatusSignal*> signals;
    std::vector<ctre::phoenix6::BaseStatusSignal*> intakeSignals = m_Intake->GetSynchronizedSignals();
    std::vector<ctre::phoenix6::BaseStatusSignal*> shooter1Signals = m_Shooter1->GetSynchronizedSignals();
    std::vector<ctre::phoenix6::BaseStatusSignal*> shooter2Signals = m_Shooter2->GetSynchronizedSignals();

    signals.insert(signals.end(), intakeSignals.begin(), intakeSignals.end());
    signals.insert(signals.end(), shooter1Signals.begin(), shooter1Signals.end());
    signals.insert(signals.end(), shooter2Signals.begin(), shooter2Signals.end());

    return signals;
}

void Shooter::ResetConstants()
{
    m_Shooter1->ConfigPID(CONSTANT("SHOOTER_P"), CONSTANT("SHOOTER_I"), CONSTANT("SHOOTER_D"));
    m_Shooter2->ConfigPID(CONSTANT("SHOOTER_P"), CONSTANT("SHOOTER_I"), CONSTANT("SHOOTER_D"));
    m_Intake->ConfigPID(CONSTANT("INTAKE_P"), CONSTANT("INTAKE_I"), CONSTANT("INTAKE_D"));
}

double Shooter::GetIntakePosition()
{
    return m_Intake->GetPosition();
}

double Shooter::GetIntakeVelocity()
{
    return m_Intake->GetVelocity();
}

double Shooter::GetIntakeAcceleration()
{
    return m_Intake->GetAcceleration();
}

double Shooter::GetIntakeCurrent()
{
    return m_Intake->GetCurrent();
}

double Shooter::GetShooterVelocity()
{
    return (m_Shooter1->GetVelocity() + m_Shooter2->GetVelocity()) / 2;
}

double Shooter::GetShooterCurrent()
{
    return m_Shooter1->GetCurrent() + m_Shooter2->GetCurrent();
}

void Shooter::Intake()
{
    if (m_IntakeState == IntakeState::IDLE || m_IntakeState == IntakeState::OUTTAKE)
    {
        m_IntakeState = IntakeState::WAIT_FOR_STOP;
    }
}

void Shooter::StopIntake()
{
    if (m_IntakeState != IntakeState::HOLD)
    {
        m_IntakeState = IntakeState::IDLE;
    }
}

void Shooter::Outtake()
{
    m_IntakeState = IntakeState::OUTTAKE;
}

void Shooter::PrimeShooter()
{
    if (m_ShooterState == ShooterState::IDLE)
    {
        m_ShooterState = ShooterState::SPIN_UP;
    }
}

void Shooter::StopShooter()
{
    if (m_IntakeState != IntakeState::SHOOT)
    {
        m_ShooterState = ShooterState::IDLE;
    }
}

void Shooter::Shoot()
{
    if (m_ShooterState == ShooterState::READY && m_IntakeState == IntakeState::HOLD)
    {
        m_IntakeState = IntakeState::SHOOT;
    }
}

void Shooter::Handle()
{
    // Intake state machine
    if (m_IntakeState == IntakeState::IDLE)
    {
        CowMotor::Control::TorqueCurrent request = {0};

        m_Intake->Set(request);
    }
    else if (m_IntakeState == IntakeState::OUTTAKE)
    {
        CowMotor::Control::DutyCycle request = {0};
        request.DutyCycle = CONSTANT("INTAKE_EXHAUST");

        m_Intake->Set(request);
    }
    else if (m_IntakeState == IntakeState::WAIT_FOR_STOP)
    {
        CowMotor::Control::TorqueCurrent request = {0};

        m_Intake->Set(request);

        if (GetIntakeVelocity() == 0)
        {
            m_IntakeState = IntakeState::DETECT;
            m_DetectStartTime = frc::Timer::GetFPGATimestamp().value();
        }
    }
    else if (m_IntakeState == IntakeState::DETECT)
    {
        CowMotor::Control::TorqueCurrent request = {0};
        request.Current = CONSTANT("INTAKE_DETECT_CURRENT");
        request.MaxDutyCycle = CONSTANT("INTAKE_DETECT_MAX_DUTY_CYCLE");
        
        m_Intake->Set(request);

        double currentTime = std::min(frc::Timer::GetFPGATimestamp().value() - m_DetectStartTime, CONSTANT("INTAKE_SPINUP_TIME"));
        double expectedAcc = (CONSTANT("INTAKE_SPINUP_MOTOR_A") * pow(currentTime, 3)) +
                             (CONSTANT("INTAKE_SPINUP_MOTOR_B") * pow(currentTime, 2)) +
                             (CONSTANT("INTAKE_SPINUP_MOTOR_C") * pow(currentTime, 1)) +
                             CONSTANT("INTAKE_SPINUP_MOTOR_D");
        double error = std::max(expectedAcc, 0.0) - GetIntakeAcceleration();

        if (error > CONSTANT("INTAKE_DETECT_ERROR_THRESHOLD"))
        {
            m_IntakeState = IntakeState::HOLD;
            m_IntakeGoalPosition = GetIntakePosition() + CONSTANT("INTAKE_MOVE_B");
        }
    }
    else if (m_IntakeState == IntakeState::HOLD)
    {
        CowMotor::Control::PositionDutyCycle request = {0};
        request.Position = m_IntakeGoalPosition;
        request.EnableFOC = true;

        m_Intake->Set(request);
    }
    else if (m_IntakeState == IntakeState::SHOOT)
    {
        CowMotor::Control::TorqueCurrent request = {0};
        request.Current = CONSTANT("INTAKE_SHOOT_CURRENT");
        request.MaxDutyCycle = CONSTANT("INTAKE_SHOOT_MAX_DUTY_CYCLE");

        m_Intake->Set(request);
    }

    // Shooter state machine
    if (m_ShooterState == ShooterState::IDLE)
    {
        CowMotor::Control::DutyCycle request = {0};

        m_Shooter1->Set(request);
        m_Shooter2->Set(request);

    }
    else if (m_ShooterState == ShooterState::SPIN_UP)
    {
        CowMotor::Control::TorqueCurrent request = {0};
        request.Current = CONSTANT("SHOOTER_SPINUP_CURRENT");
        request.MaxDutyCycle = CONSTANT("SHOOTER_SPINUP_MAX_DUTY_CYCLE");

        m_Shooter1->Set(request);
        m_Shooter2->Set(request);

        if (GetShooterVelocity() > CONSTANT("SHOOTER_SPINUP_VEL_THRESHOLD"))
        {
            m_ShooterState = ShooterState::READY;
        }
    }
    else if (m_ShooterState == ShooterState::READY)
    {
        CowMotor::Control::TorqueCurrent request = {0};
        request.Current = CONSTANT("SHOOTER_SPINUP_CURRENT");
        request.MaxDutyCycle = CONSTANT("SHOOTER_SPINUP_MAX_DUTY_CYCLE");

        m_Shooter1->Set(request);
        m_Shooter2->Set(request);

        if (GetShooterVelocity() < CONSTANT("SHOOTER_READY_VEL_THRESHOLD"))
        {
            m_ShooterState = ShooterState::SPIN_UP;
        }
    }
}
   