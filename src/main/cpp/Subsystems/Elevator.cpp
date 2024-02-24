#include "Elevator.h"

Elevator::Elevator(const int motorID1, const int motorID2)
{
    m_Motor1 = std::make_unique<CowMotor::TalonFX>(motorID1, "cowbus");
    m_Motor1->ConfigPositivePolarity(CowMotor::Direction::COUNTER_CLOCKWISE);
    m_Motor1->ConfigNeutralMode(CowMotor::NeutralMode::BRAKE);

    m_Motor2 = std::make_unique<CowMotor::TalonFX>(motorID2, "cowbus");
    m_Motor2->ConfigPositivePolarity(CowMotor::Direction::COUNTER_CLOCKWISE);
    m_Motor2->ConfigNeutralMode(CowMotor::NeutralMode::BRAKE);

    m_PositionRequest.EnableFOC = true;
    m_PositionRequest.Position = 0;
    m_PositionRequest.FeedForward = 0;

    m_FollowerRequest.MasterID = motorID1;
    m_FollowerRequest.OpposeMasterDirection = false;

    m_TargetExtensionLength = 0;

    ResetConstants();
}

std::vector<ctre::phoenix6::BaseStatusSignal*> Elevator::GetSynchronizedSignals()
{
    std::vector<ctre::phoenix6::BaseStatusSignal*> signals;
    std::vector<ctre::phoenix6::BaseStatusSignal*> motor1Signals = m_Motor1->GetSynchronizedSignals();
    std::vector<ctre::phoenix6::BaseStatusSignal*> motor2Signals = m_Motor2->GetSynchronizedSignals();

    signals.insert(signals.end(), motor1Signals.begin(), motor1Signals.end());
    signals.insert(signals.end(), motor2Signals.begin(), motor2Signals.end());

    return signals;
}

void Elevator::ResetConstants()
{
    m_Motor1->ConfigPID(CONSTANT("ELEVATOR_KP"), CONSTANT("ELEVATOR_KI"), CONSTANT("ELEVATOR_KD"));

    m_Motor1->ConfigMotionMagic(
        CONSTANT("ELEVATOR_KV") / CONSTANT("ELEVATOR_INCHES_PER_TURN"),
        CONSTANT("ELEVATOR_KA") / CONSTANT("ELEVATOR_INCHES_PER_TURN"));
}

double Elevator::GetPosition()
{
    return m_Motor1->GetPosition() * CONSTANT("ELEVATOR_INCHES_PER_TURN");
}

double Elevator::GetVelocity()
{
    return m_Motor1->GetVelocity() * CONSTANT("ELEVATOR_INCHES_PER_TURN");
}

double Elevator::GetAcceleration()
{
    return m_Motor1->GetAcceleration() * CONSTANT("ELEVATOR_INCHES_PER_TURN");
}

double Elevator::GetCurrent()
{
    return m_Motor1->GetCurrent() + m_Motor2->GetCurrent();
}

void Elevator::SetExtension(double extensionLength, double pivotSetpoint)
{
    if (pivotSetpoint <= CONSTANT("PREVENT_EXTENSION_UNDER_ANGLE"))
    {
        extensionLength = CONSTANT("ELEVATOR_MIN_EXTENSION");
    }
    m_TargetExtensionLength = std::clamp(extensionLength, CONSTANT("ELEVATOR_MIN_EXTENSION"), CONSTANT("ELEVATOR_MAX_EXTENSION"));
}

void Elevator::Handle()
{
    m_PositionRequest.Position = m_TargetExtensionLength / CONSTANT("ELEVATOR_INCHES_PER_TURN");

    m_Motor1->Set(m_PositionRequest);
    m_Motor2->Set(m_FollowerRequest);
}