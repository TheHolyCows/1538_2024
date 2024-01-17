#include "Elevator.h"

#include <frc/trajectory/TrapezoidProfile.h>
#include <units/velocity.h>

Elevator::Elevator(const int MotorId)
{
    m_ElevatorMotor = std::make_shared<CowLib::CowMotorController>(MotorId, CowMotor::PHOENIX_V6);
    m_ElevatorMotor->SetNeutralMode(CowMotor::BRAKE);

    ResetConstants();
}

void Elevator::RequestPosition(double pos)
{
   m_MotorRequest.Position = pos * CONSTANT("ELEVATOR_GEAR_RATIO");

    if (GetPosition() < GetSetpoint())
    {
        m_MotorRequest.FeedForward = CONSTANT("ELEVATOR_FF");
    }
    else
    {
        m_MotorRequest.FeedForward = 0;
    };
}

double Elevator::GetSetpoint()
{
    return m_MotorRequest.Position / CONSTANT("ELEVATOR_GEAR_RATIO");
}

bool Elevator::AtTarget()
{
    return fabs(GetPosition() - GetSetpoint()) < CONSTANT("ELEVATOR_TOLERANCE");
}

double Elevator::GetPosition()
{
    return m_ElevatorMotor->GetPosition() / CONSTANT("ELEVATOR_GEAR_RATIO");
}

void Elevator::ResetConstants()
{
    m_ElevatorMotor->SetPID(CONSTANT("ELEVATOR_DOWN_P"),
                             CONSTANT("ELEVATOR_DOWN_I"),
                             CONSTANT("ELEVATOR_DOWN_D"),
                             CONSTANT("ELEVATOR_DOWN_F"));
    m_ElevatorMotor->SetMotionMagic(CONSTANT("ELEVATOR_DOWN_V"), CONSTANT("ELEVATOR_DOWN_A"));
}

void Elevator::Handle()
{
    m_ElevatorMotor->Set(m_MotorRequest);
}

void Elevator::UsePIDSet(Elevator::PIDSet set)
{
    if (set != m_PrevPIDSet)
    {
        switch (set)
        {
        case EXTENDING :
            m_ElevatorMotor->SetPID(CONSTANT("ELEVATOR_UP_P"),
                                     CONSTANT("ELEVATOR_UP_I"),
                                     CONSTANT("ELEVATOR_UP_D"),
                                     CONSTANT("ELEVATOR_UP_F"));
            m_ElevatorMotor->SetMotionMagic(CONSTANT("ELEVATOR_UP_V"), CONSTANT("ELEVATOR_UP_A"));
           std::cout << "UP elevator constants\n";

            break;
        case RETRACTING :
            m_ElevatorMotor->SetPID(CONSTANT("ELEVATOR_DOWN_P"),
                                     CONSTANT("ELEVATOR_DOWN_I"),
                                     CONSTANT("ELEVATOR_DOWN_D"),
                                     CONSTANT("ELEVATOR_DOWN_F"));
            m_ElevatorMotor->SetMotionMagic(CONSTANT("ELEVATOR_DOWN_V"), CONSTANT("ELEVATOR_DOWN_A"));
            std::cout << "DOWN telescope constants\n";

            break;
        default :
            break;
        }
    }

    m_PrevPIDSet = set;
}
