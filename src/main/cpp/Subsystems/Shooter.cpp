#include "Shooter.h"

Shooter::Shooter(const int shooterID1, const int shooterID2, const int intakeID1, const int intakeID2, const int wristID) 
{

    m_Shooter1 = std::make_unique<CowMotor::TalonFX>(shooterID1, "cowdrive");
    m_Shooter2 = std::make_unique<CowMotor::TalonFX>(shooterID2, "cowdrive");
    m_Shooter1->ConfigNeutralMode(CowMotor::NeutralMode::COAST);
    m_Shooter2->ConfigNeutralMode(CowMotor::NeutralMode::COAST);

    m_Shooter2->ConfigPositivePolarity(CowMotor::Direction::CLOCKWISE);
    
    m_Intake1  = std::make_unique<CowMotor::TalonFX>(intakeID1, "cowdrive");
    m_Intake2  = std::make_unique<CowMotor::TalonFX>(intakeID2, "cowdrive");
    m_Intake1->ConfigNeutralMode(CowMotor::NeutralMode::BRAKE);
    m_Intake2->ConfigNeutralMode(CowMotor::NeutralMode::BRAKE);

    // m_Wrist  = std::make_unique<CowMotor::TalonFX>(wristID, "cowdrive");
    // m_Wrist->ConfigNeutralMode(CowMotor::NeutralMode::COAST);

    // double m_WristPosition = 0;

    m_IntakeState = IntakeState::IDLE;
    m_ShooterState = ShooterState::IDLE;
}

std::vector<ctre::phoenix6::BaseStatusSignal*> Shooter::GetSynchronizedSignals()
{
    std::vector<ctre::phoenix6::BaseStatusSignal*> signals;
    std::vector<ctre::phoenix6::BaseStatusSignal*> intake1Signals = m_Intake1->GetSynchronizedSignals();
    std::vector<ctre::phoenix6::BaseStatusSignal*> intake2Signals = m_Intake2->GetSynchronizedSignals();
    std::vector<ctre::phoenix6::BaseStatusSignal*> shooter1Signals = m_Shooter1->GetSynchronizedSignals();
    std::vector<ctre::phoenix6::BaseStatusSignal*> shooter2Signals = m_Shooter2->GetSynchronizedSignals();

    signals.insert(signals.end(), intake1Signals.begin(), intake1Signals.end());
    signals.insert(signals.end(), intake2Signals.begin(), intake2Signals.end());
    signals.insert(signals.end(), shooter1Signals.begin(), shooter1Signals.end());
    signals.insert(signals.end(), shooter2Signals.begin(), shooter2Signals.end());

    return signals;
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
    // m_Wrist->ConfigPID(CONSTANT("WRIST_P"), CONSTANT("WRIST_I"), CONSTANT("WRIST_D"), CONSTANT("WRIST_F"));
    // m_Wrist->ConfigMotionMagic(CONSTANT("WRIST_V"), CONSTANT("WRIST_A"));
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
    // m_WristControlRequest.Position = CowLib::Conversions::DegreesToFalcon(angle, CONSTANT("WRIST_GEAR_RATIO")) * -1;
}

double Shooter::GetWristSetpoint()
{
    // return CowLib::Conversions::FalconToDegrees(m_WristControlRequest.Position, CONSTANT("WRIST_GEAR_RATIO")) * -1;
}

bool Shooter::WristAtTarget()
{
    return fabs(GetWristSetpoint() - GetWristAngle() < CONSTANT("WRIST_TOLERANCE"));
}

double Shooter::GetWristAngle()
{
    // return CowLib::Conversions::FalconToDegrees(m_Wrist->GetPosition(), CONSTANT("WRIST_GEAR_RATIO")) * -1;
}

void Shooter::Intake()
{
    if(m_IntakeState == IntakeState::IDLE || m_IntakeState == IntakeState::OUTTAKE)
    {
        m_IntakeState = IntakeState::WAIT_FOR_STOP;
    }
}

void Shooter::StopIntake()
{
    if(m_IntakeState != IntakeState::HOLD)
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
    if(m_ShooterState == ShooterState::IDLE)
    {
        m_ShooterState = ShooterState::SPIN_UP;
    }
}

void Shooter::StopShooter()
{
    if(m_IntakeState != IntakeState::SHOOT)
    {
        m_ShooterState = ShooterState::IDLE;
    }
}

void Shooter::Shoot()
{
    if(m_ShooterState == ShooterState::READY && m_IntakeState == IntakeState::HOLD)
    {
        m_IntakeState = IntakeState::SHOOT;
    }
}

void Shooter::Handle()
{
    double totalCurrent = m_Intake1->GetCurrent() + m_Intake2->GetCurrent();
    double meanIntakeVel = (m_Intake1->GetVelocity() + m_Intake2->GetVelocity()) / 2;
    double meanIntakeAcc = (m_Intake1->GetAcceleration() + m_Intake2->GetAcceleration()) / 2;
    double meanShooterVel = (m_Shooter1->GetVelocity() + m_Shooter2->GetVelocity()) / 2;

    if (m_IntakeState == IntakeState::IDLE)
    {
        CowMotor::Control::TorqueCurrent request = {0};

        m_Intake1->Set(request);
        m_Intake2->Set(request);
    }
    else if(m_IntakeState == IntakeState::OUTTAKE)
    {
        CowMotor::Control::DutyCycle request = {0};
        request.DutyCycle = CONSTANT("INTAKE_EXHAUST");
        m_Intake1->Set(request);
        m_Intake2->Set(request);
    }
    else if (m_IntakeState == IntakeState::WAIT_FOR_STOP)
    {
        CowMotor::Control::DutyCycle request = {0};

        m_Intake1->Set(request);
        m_Intake2->Set(request);

        if (meanIntakeVel == 0)
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
        
        m_Intake1->Set(request);
        m_Intake2->Set(request);

        double currentTime = frc::Timer::GetFPGATimestamp().value() - m_DetectStartTime;
        double expectedAcc = 0;

        // printf("%f\t%f\n", currentTime, meanIntakeAcc);

        // if (currentTime > CONSTANT("INTAKE_SPINUP_GRACE_PERIOD"))
        // {
            if (currentTime < CONSTANT("INTAKE_SPINUP_TIME"))
            {
                expectedAcc = (CONSTANT("INTAKE_SPINUP_MOTOR_A") * pow(currentTime, 3)) +
                            (CONSTANT("INTAKE_SPINUP_MOTOR_B") * pow(currentTime, 2)) +
                            (CONSTANT("INTAKE_SPINUP_MOTOR_C") * pow(currentTime, 1)) +
                            CONSTANT("INTAKE_SPINUP_MOTOR_D");
            }
            else
            {
                expectedAcc = (CONSTANT("INTAKE_SPINUP_MOTOR_A") * pow(CONSTANT("INTAKE_SPINUP_TIME"), 3)) +
                            (CONSTANT("INTAKE_SPINUP_MOTOR_B") * pow(CONSTANT("INTAKE_SPINUP_TIME"), 2)) +
                            (CONSTANT("INTAKE_SPINUP_MOTOR_C") * pow(CONSTANT("INTAKE_SPINUP_TIME"), 1)) +
                            CONSTANT("INTAKE_SPINUP_MOTOR_D");
            }

            double error = std::max(expectedAcc, 0.0) - meanIntakeAcc;

            if (error > CONSTANT("INTAKE_DETECT_ERROR_THRESHOLD"))
            {
                m_IntakeState = IntakeState::HOLD;
                m_Intake1GoalPosition = m_Intake1->GetPosition() + (CONSTANT("INTAKE_MOVE_M") * meanIntakeVel) + CONSTANT("INTAKE_MOVE_B");
                m_Intake2GoalPosition = m_Intake2->GetPosition() + (CONSTANT("INTAKE_MOVE_M") * meanIntakeVel) + CONSTANT("INTAKE_MOVE_B");
            }
        // }
    }
    else if (m_IntakeState == IntakeState::HOLD)
    {
        CowMotor::Control::PositionDutyCycle request1 = {0};
        CowMotor::Control::PositionDutyCycle request2 = {0};
        request1.Position = m_Intake1GoalPosition;
        request2.Position = m_Intake2GoalPosition;
        request2.EnableFOC = true;

        m_Intake1->Set(request1);
        m_Intake2->Set(request2);

    }
    else if (m_IntakeState == IntakeState::SHOOT)
    {
        CowMotor::Control::TorqueCurrent request = {0};
        request.Current = CONSTANT("INTAKE_SHOOT_CURRENT");
        request.MaxDutyCycle = CONSTANT("INTAKE_SHOOT_MAX_DUTY_CYCLE");

        m_Intake1->Set(request);
        m_Intake2->Set(request);
    }

    if(m_ShooterState == ShooterState::IDLE)
    {
        CowMotor::Control::DutyCycle request = {0};

        m_Shooter1->Set(request);
        m_Shooter2->Set(request);

    }
    else if(m_ShooterState == ShooterState::SPIN_UP)
    {
        CowMotor::Control::TorqueCurrent request = {0};
        request.Current = CONSTANT("SHOOTER_SPINUP_CURRENT");
        request.MaxDutyCycle = CONSTANT("SHOOTER_SPINUP_MAX_DUTY_CYCLE");

        m_Shooter1->Set(request);
        m_Shooter2->Set(request);

        if(meanShooterVel > CONSTANT("SHOOTER_SPINUP_VEL_THRESHOLD"))
        {
            m_ShooterState = ShooterState::READY;
        }

    }
    else if(m_ShooterState == ShooterState::READY)
    {
        CowMotor::Control::TorqueCurrent request = {0};
        request.Current = CONSTANT("SHOOTER_SPINUP_CURRENT");
        request.MaxDutyCycle = CONSTANT("SHOOTER_SPINUP_MAX_DUTY_CYCLE");

        m_Shooter1->Set(request);
        m_Shooter2->Set(request);

        if(meanShooterVel < CONSTANT("SHOOTER_READY_VEL_THRESHOLD"))
        {
            m_ShooterState = ShooterState::SPIN_UP;
        }
    }

    // if(m_Shooter1)
    // {
    //     m_Shooter1->Set(m_ShooterControlRequest);
    // }

    // if(m_Shooter2)
    // {
    //     m_Shooter2->Set(m_ShooterControlRequest);
    // }

    // if(m_Intake1)
    // {
    //     m_Intake1->Set(m_IntakeControlRequest);
    // }

    // if(m_Intake2)
    // {
    //     m_Intake2->Set(m_IntakeControlRequest);
    // }

    // if (m_Wrist)
    // {
    //     m_Wrist->Set(m_WristControlRequest);
    // }
}
   