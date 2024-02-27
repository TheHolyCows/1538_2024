#include "Shooter.h"

Shooter::Shooter(const int shooterID1, const int shooterID2, const int intakeID)
{
    m_Shooter1 = std::make_unique<CowMotor::TalonFX>(shooterID1, "cowbus");
    m_Shooter1->ConfigPositivePolarity(CowMotor::Direction::CLOCKWISE);
    m_Shooter1->ConfigNeutralMode(CowMotor::NeutralMode::COAST);

    m_Shooter2 = std::make_unique<CowMotor::TalonFX>(shooterID2, "cowbus");
    m_Shooter2->ConfigPositivePolarity(CowMotor::Direction::CLOCKWISE);
    m_Shooter2->ConfigNeutralMode(CowMotor::NeutralMode::COAST);
    
    m_Intake = std::make_unique<CowMotor::TalonFX>(intakeID, "cowbus");
    m_Intake->ConfigPositivePolarity(CowMotor::Direction::COUNTER_CLOCKWISE);
    m_Intake->ConfigNeutralMode(CowMotor::NeutralMode::BRAKE);

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

void Shooter::CalibrateIntake()
{
    if (m_IntakeState != IntakeState::CALIBRATION_START &&
        m_IntakeState != IntakeState::CALIBRATION_COLLECT &&
        m_IntakeState != IntakeState::CALIBRATION_END)
    {
        m_IntakeState = IntakeState::CALIBRATION_START;
    }
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
    if (m_IntakeState == IntakeState::IDLE || m_IntakeState == IntakeState::EXHAUST)
    {
        m_IntakeState = IntakeState::WAIT_FOR_STOP;
    }
}

void Shooter::Exhaust()
{
    m_IntakeState = IntakeState::EXHAUST;
}

void Shooter::StopIntake()
{
    if (m_IntakeState != IntakeState::HOLD)
    {
        m_IntakeState = IntakeState::IDLE;
    }
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
    if (m_IntakeState == IntakeState::HOLD && m_ShooterState == ShooterState::READY)
    {
        m_IntakeState = IntakeState::SHOOT;
    }
}

void Shooter::Handle()
{
    // Intake state machine
    if (m_IntakeState == IntakeState::CALIBRATION_START)
    {
        m_IntakeCalibrationStartTime = frc::Timer::GetFPGATimestamp().value();
        m_IntakeState = IntakeState::CALIBRATION_COLLECT;
    }
    else if (m_IntakeState == IntakeState::CALIBRATION_COLLECT)
    {
        // Run the intake rollers with the detection parameters
        CowMotor::Control::TorqueCurrent request = {};
        request.Current = CONSTANT("INTAKE_DETECT_CURRENT");
        request.MaxDutyCycle = CONSTANT("INTAKE_DETECT_MAX_DUTY_CYCLE");

        m_Intake->Set(request);

        // Collect data
        double elapsed = frc::Timer::GetFPGATimestamp().value() - m_IntakeCalibrationStartTime;

        // TODO: Save this to a file instead of dumping it in stdout
        printf("%f,%f,%f", elapsed, GetIntakeAcceleration(), GetIntakeCurrent());

        if (elapsed > CONSTANT("INTAKE_CALIBRATION_PERIOD"))
        {
            m_IntakeState = IntakeState::CALIBRATION_END;
        }
    }
    else if (m_IntakeState == IntakeState::CALIBRATION_END)
    {
        CowMotor::Control::TorqueCurrent request = {};

        m_Intake->Set(request);
    }
    else if (m_IntakeState == IntakeState::IDLE)
    {
        CowMotor::Control::DutyCycle request = {};

        m_Intake->Set(request);
    }
    else if (m_IntakeState == IntakeState::EXHAUST)
    {
        CowMotor::Control::DutyCycle request = {};
        request.EnableFOC = true;
        request.DutyCycle = CONSTANT("INTAKE_EXHAUST");

        m_Intake->Set(request);
    }
    else if (m_IntakeState == IntakeState::WAIT_FOR_STOP)
    {
        CowMotor::Control::TorqueCurrent request = {};
        m_Intake->Set(request);

        if (GetIntakeVelocity() == 0)
        {
            m_IntakeState = IntakeState::DETECT;
            m_DetectStartTime = frc::Timer::GetFPGATimestamp().value();
        }
    }
    else if (m_IntakeState == IntakeState::DETECT)
    {
        CowMotor::Control::TorqueCurrent request = {};
        request.Current = CONSTANT("INTAKE_DETECT_CURRENT");
        request.MaxDutyCycle = CONSTANT("INTAKE_DETECT_MAX_DUTY_CYCLE");

        m_Intake->Set(request);

        double currentTime = std::min(frc::Timer::GetFPGATimestamp().value() - m_DetectStartTime, CONSTANT("INTAKE_SPINUP_TIME"));
        double expectedAcc = (CONSTANT("INTAKE_SPINUP_CURVE_A") * pow(currentTime, 3)) +
                             (CONSTANT("INTAKE_SPINUP_CURVE_B") * pow(currentTime, 2)) +
                             (CONSTANT("INTAKE_SPINUP_CURVE_C") * pow(currentTime, 1)) +
                             CONSTANT("INTAKE_SPINUP_CURVE_D");
        double error = std::max(expectedAcc, 0.0) - GetIntakeAcceleration();

        if (error > CONSTANT("INTAKE_DETECT_ERROR_THRESHOLD"))
        {
            m_IntakeState = IntakeState::HOLD;
            m_IntakeGoalPosition = GetIntakePosition() + CONSTANT("INTAKE_MOVE_DISTANCE");
        }
    }
    else if (m_IntakeState == IntakeState::HOLD)
    {
        CowMotor::Control::PositionDutyCycle request = {};
        request.EnableFOC = true;
        request.Position = m_IntakeGoalPosition;

        m_Intake->Set(request);
    }
    else if (m_IntakeState == IntakeState::SHOOT)
    {
        CowMotor::Control::TorqueCurrent request = {};
        request.Current = CONSTANT("INTAKE_SHOOT_CURRENT");
        request.MaxDutyCycle = CONSTANT("INTAKE_SHOOT_MAX_DUTY_CYCLE");

        m_Intake->Set(request);
    }

    // Shooter state machine
    if (m_ShooterState == ShooterState::IDLE)
    {
        CowMotor::Control::DutyCycle request = {};

        m_Shooter1->Set(request);
        m_Shooter2->Set(request);
    }
    else if (m_ShooterState == ShooterState::SPIN_UP)
    {
        CowMotor::Control::TorqueCurrent request = {};
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
        CowMotor::Control::TorqueCurrent request = {};
        request.Current = CONSTANT("SHOOTER_SPINUP_CURRENT");
        request.MaxDutyCycle = CONSTANT("SHOOTER_SPINUP_MAX_DUTY_CYCLE");

        m_Shooter1->Set(request);
        m_Shooter2->Set(request);

        if (GetShooterVelocity() < CONSTANT("SHOOTER_READY_VEL_THRESHOLD"))
        {
            m_ShooterState = ShooterState::SPIN_UP;
        }
    }

    if (m_CycleCount++ % 100 == 0) // log every (100 * 10ms) = 1 sec
    {
        CowLib::CowLogger::GetInstance()->LogState(CowLib::CowLogger::SHOOTER, (uint16_t) m_ShooterState);
        CowLib::CowLogger::GetInstance()->LogState(CowLib::CowLogger::INTAKE, (uint16_t) m_IntakeState);
        m_CycleCount = 1;
    }
}
   