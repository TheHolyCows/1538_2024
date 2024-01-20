#include "Elevator.h"

Elevator::Elevator(const int emotorID, const int pmotorID)
{
    m_ElevatorMotor = std::make_shared<CowLib::CowMotorController>(emotorID, CowMotor::PHOENIX_V6);
    m_PivotMotor = std::make_shared<CowLib::CowMotorController>(pmotorID, CowMotor::PHOENIX_V6);
    m_ElevatorMotor->SetNeutralMode(CowMotor::BRAKE);
    m_PivotMotor->SetNeutralMode(CowMotor::BRAKE);

    m_TargetAngle = 0;
    m_TickCount   = 0;

    m_ClimberState = ST_DEFAULT;

    ResetConstants();
}

void Elevator::RequestElevatorPosition(double pos)
{
   m_ElevatorMotorRequest.Position = pos * CONSTANT("ELEVATOR_GEAR_RATIO");

    if (GetElevatorPosition() < GetElevatorSetpoint())
    {
        m_ElevatorMotorRequest.FeedForward = CONSTANT("ELEVATOR_FF");
    }
    else
    {
        m_ElevatorMotorRequest.FeedForward = 0;
    };
}

double Elevator::GetElevatorSetpoint()
{
    return m_ElevatorMotorRequest.Position / CONSTANT("ELEVATOR_GEAR_RATIO");
}

bool Elevator::ElevatorAtTarget()
{
    return fabs(GetElevatorPosition() - GetElevatorSetpoint()) < CONSTANT("ELEVATOR_TOLERANCE");
}

double Elevator::GetElevatorPosition()
{
    return m_ElevatorMotor->GetPosition() / CONSTANT("ELEVATOR_GEAR_RATIO");
}

void Elevator::RequestPivotAngle(double angle)
{
    m_PivotMotorRequest.Position = CowLib::Conversions::DegreesToFalcon(angle, CONSTANT("PIVOT_GEAR_RATIO"));
}

double Elevator::GetPivotSetpoint()
{
    return CowLib::Conversions::FalconToDegrees(m_TargetAngle, CONSTANT("PIVOT_GEAR_RATIO"));
}

bool Elevator::PivotAtTarget()
{
    return fabs(GetPivotSetpoint() - GetPivotAngle() < CONSTANT("PIVOT_TOLERANCE"));
}

double Elevator::GetPivotAngle()
{
    return CowLib::Conversions::FalconToDegrees(m_PivotMotor->GetPosition(),
                                                CONSTANT("PIVOT_GEAR_RATIO"));
}

void Elevator::UpdatePID(double armExt)
{
    double p
        = CONSTANT("PIVOT_P_BASE") + (CONSTANT("PIVOT_P_EXTENSION") * armExt) + (CONSTANT("PIVOT_P_ANGLE") * armExt);

    m_PivotMotor->SetPID(p,CONSTANT("PIVOT_I"),CONSTANT("PIVOT_D"),CONSTANT("PIVOT_F"));
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
    m_ElevatorMotor->Set(m_ElevatorMotorRequest);
    m_PivotMotor->Set(m_PivotMotorRequest);
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
        //    std::cout << "UP elevator constants\n";

            break;
        case RETRACTING :
            m_ElevatorMotor->SetPID(CONSTANT("ELEVATOR_DOWN_P"),
                                     CONSTANT("ELEVATOR_DOWN_I"),
                                     CONSTANT("ELEVATOR_DOWN_D"),
                                     CONSTANT("ELEVATOR_DOWN_F"));
            m_ElevatorMotor->SetMotionMagic(CONSTANT("ELEVATOR_DOWN_V"), CONSTANT("ELEVATOR_DOWN_A"));
            // std::cout << "DOWN telescope constants\n";

            break;
        default :
            break;
        }
    }

    m_PrevPIDSet = set;
}
