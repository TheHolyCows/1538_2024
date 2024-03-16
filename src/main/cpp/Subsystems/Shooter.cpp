#include "Shooter.h"

Shooter::Shooter(const int shooterID1, const int shooterID2, const int intakeID, const int cancoderID, Vision *vision)
{
    m_Shooter1 = std::make_unique<CowMotor::TalonFX>(shooterID1, "cowbus");
    m_Shooter1->ConfigPositivePolarity(CowMotor::Direction::CLOCKWISE);
    m_Shooter1->ConfigNeutralMode(CowMotor::NeutralMode::BRAKE);
    m_Shooter1->ConfigStatorCurrentLimit(400.0);

    m_Shooter2 = std::make_unique<CowMotor::TalonFX>(shooterID2, "cowbus");
    m_Shooter2->ConfigPositivePolarity(CowMotor::Direction::CLOCKWISE);
    m_Shooter2->ConfigNeutralMode(CowMotor::NeutralMode::BRAKE);
    m_Shooter2->ConfigStatorCurrentLimit(400.0);

    m_Intake = std::make_unique<CowMotor::TalonFX>(intakeID, "cowbus");
    m_Intake->ConfigPositivePolarity(CowMotor::Direction::CLOCKWISE);
    m_Intake->ConfigNeutralMode(CowMotor::NeutralMode::BRAKE);

    m_CANCoder = std::make_unique<CowLib::CowCANCoder>(cancoderID, "cowbus");

    m_Vision = vision;

    m_IntakeState = IntakeState::IDLE;
    m_ShooterState = ShooterState::IDLE;

    m_CycleCount = 1;

    ResetConstants();
}

std::vector<ctre::phoenix6::BaseStatusSignal*> Shooter::GetSynchronizedSignals()
{
    std::vector<ctre::phoenix6::BaseStatusSignal*> signals;
    std::vector<ctre::phoenix6::BaseStatusSignal*> intakeSignals = m_Intake->GetSynchronizedSignals();
    std::vector<ctre::phoenix6::BaseStatusSignal*> shooter1Signals = m_Shooter1->GetSynchronizedSignals();
    std::vector<ctre::phoenix6::BaseStatusSignal*> shooter2Signals = m_Shooter2->GetSynchronizedSignals();
    std::vector<ctre::phoenix6::BaseStatusSignal*> canCoderSignals = m_CANCoder->GetSynchronizedSignals();

    signals.insert(signals.end(), intakeSignals.begin(), intakeSignals.end());
    signals.insert(signals.end(), shooter1Signals.begin(), shooter1Signals.end());
    signals.insert(signals.end(), shooter2Signals.begin(), shooter2Signals.end());
    signals.insert(signals.end(), canCoderSignals.begin(), canCoderSignals.end());

    return signals;
}

void Shooter::ResetConstants()
{
    m_Shooter1->ConfigPID(CONSTANT("SHOOTER_P"), CONSTANT("SHOOTER_I"), CONSTANT("SHOOTER_D"), 0.0, CONSTANT("SHOOTER_F"));
    m_Shooter2->ConfigPID(CONSTANT("SHOOTER_P"), CONSTANT("SHOOTER_I"), CONSTANT("SHOOTER_D"), 0.0, CONSTANT("SHOOTER_F"));
    m_Intake->ConfigPID(CONSTANT("INTAKE_P"), CONSTANT("INTAKE_I"), CONSTANT("INTAKE_D"));
}

Shooter::IntakeState Shooter::GetIntakeState()
{
    return m_IntakeState;
}

Shooter::ShooterState Shooter::GetShooterState()
{
    return m_ShooterState;
}

void Shooter::UpdateIntakeState(IntakeState state)
{
    m_IntakeState = state;
}

void Shooter::UpdateShooterState(ShooterState state)
{
    m_ShooterState = state;
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

bool Shooter::IsReady()
{
    if (m_ShooterState == ShooterState::READY)
    {
        return true;
    }
    return false;
}

void Shooter::StopIntake()
{
    if (m_IntakeState != IntakeState::DETECT_HOLD)
    {
        m_IntakeState = IntakeState::IDLE;
    }
}

void Shooter::Intake()
{
    if (m_IntakeState != IntakeState::DETECT_ACTIVE &&
        m_IntakeState != IntakeState::DETECT_HOLD &&
        m_IntakeState != IntakeState::SHOOT)
    {
        m_IntakeState = IntakeState::DETECT_ACTIVE;
    }
}

void Shooter::Exhaust()
{
    m_IntakeState = IntakeState::EXHAUST;
    m_ShooterState = ShooterState::EXHAUST;
}

void Shooter::PrimeShooter(double rps)
{
    m_ShooterRPS = rps;

    if (m_ShooterState != ShooterState::SPIN_UP &&
        m_ShooterState != ShooterState::READY)
    {
        m_ShooterState = ShooterState::SPIN_UP;

        m_ShooterStartTime = 0.0;
    }
}

void Shooter::StopShooter()
{
    m_ShooterState = ShooterState::IDLE;
}

void Shooter::Shoot()
{
    if(IsReady())
    {
        m_IntakeState = IntakeState::SHOOT;
    }
}

void Shooter::Handle()
{
    // Intake state machine
    switch (m_IntakeState)
    {
        // Idle
        case IntakeState::IDLE:
        {
            CowMotor::Control::TorqueCurrent request = {};
            m_Intake->Set(request);

            break;
        }

        // Detect
        case IntakeState::DETECT_ACTIVE:
        {
            CowMotor::Control::TorqueCurrent request = {};
            request.MaxDutyCycle = CONSTANT("INTAKE_MAX_DUTY_CYCLE");
            request.Current = CONSTANT("INTAKE_SPINUP_CURRENT");

            m_Intake->Set(request);

            if (!m_CANCoder->GetMagnetIsHealthy())
            {
                m_IntakeState = IntakeState::DETECT_HOLD;
                m_IntakeGoalPosition = GetIntakePosition() + CONSTANT("INTAKE_MOVE_DISTANCE");
            }

            break;
        }
        case IntakeState::DETECT_HOLD:
        {
            CowMotor::Control::PositionDutyCycle request = {};
            request.EnableFOC = true;
            request.Position = m_IntakeGoalPosition;

            m_Intake->Set(request);

            if (m_CANCoder->GetMagnetIsHealthy())
            {
                // m_IntakeState = IntakeState::DETECT_ACTIVE;
            }

            break;
        }

        // Shoot
        case IntakeState::SHOOT:
        {
            CowMotor::Control::TorqueCurrent request = {};
            request.Current = CONSTANT("INTAKE_SHOOT_CURRENT");
            request.MaxDutyCycle = CONSTANT("INTAKE_SHOOT_MAX_DUTY_CYCLE");

            m_Intake->Set(request);

            break;
        }

        // Exhaust
        case IntakeState::EXHAUST:
        {
            CowMotor::Control::TorqueCurrent request = {};
            request.Current = CONSTANT("INTAKE_EXHAUST_CURRENT");
            request.MaxDutyCycle = CONSTANT("INTAKE_EXHAUST_MAX_DUTY_CYCLE");

            m_Intake->Set(request);

            break;
        }
    }

    // Shooter state machine
    switch (m_ShooterState)
    {
        case ShooterState::IDLE:
        {
            CowMotor::Control::DutyCycle request = {};

            m_Shooter1->Set(request);
            m_Shooter2->Set(request);

            break;
        }
        case ShooterState::SPIN_UP:
        {
            if (m_ShooterStartTime == 0.0)
            {
                m_ShooterStartTime = frc::Timer::GetFPGATimestamp().value();
            }

            double elapsed = frc::Timer::GetFPGATimestamp().value() - m_ShooterStartTime;

            CowMotor::Control::VelocityDutyCycle request = {};
            request.EnableFOC = true;
            request.Velocity = m_ShooterRPS;

            m_Shooter1->Set(request);
            m_Shooter2->Set(request);

            if (GetShooterVelocity() > m_ShooterRPS * CONSTANT("SHOOTER_SPINUP_VEL_TOLERANCE") || elapsed > CONSTANT("SHOOTER_SPINIUP_TIMEOUT"))
            {
                m_ShooterState = ShooterState::READY;
            }

            break;
        }
        case ShooterState::READY:
        {
            CowMotor::Control::VelocityDutyCycle request = {};
            request.EnableFOC = true;
            request.Velocity = m_ShooterRPS;

            m_Shooter1->Set(request);
            m_Shooter2->Set(request);

            if (GetShooterVelocity() < m_ShooterRPS * CONSTANT("SHOOTER_SPINUP_VEL_TOLERANCE"))
            {
                m_ShooterState = ShooterState::SPIN_UP;
            }

            break;
        }
        case ShooterState::EXHAUST:
        {
            CowMotor::Control::TorqueCurrent request = {};
            request.Current = CONSTANT("SHOOTER_EXHAUST_CURRENT");
            request.MaxDutyCycle = CONSTANT("SHOOTER_EXHAUST_MAX_DUTY_CYCLE");

            m_Shooter1->Set(request);
            m_Shooter2->Set(request);

            break;
        }
    }

    if (m_CycleCount++ % 100 == 0) // log every (100 * 10ms) = 1 sec
    {
        CowLib::CowLogger::GetInstance()->LogState(CowLib::CowLogger::SHOOTER, (uint16_t) m_ShooterState);
        CowLib::CowLogger::GetInstance()->LogState(CowLib::CowLogger::INTAKE, (uint16_t) m_IntakeState);
        m_CycleCount = 1;
    }
}