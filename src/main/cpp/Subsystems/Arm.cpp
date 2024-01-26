#include "Arm.h"

Arm::Arm(const int elevatorMotorID, const int pivotMotorID, const int shooterID1, const int shooterID2, const int intakeID1, const int intakeID2, const int wristID)
{
    // m_Elevator = new Elevator(const int elevatorMotorID, const int pivotMotorID);
    // m_Shooter = new Shooter(const int shooterID1, const int shooterID2, const int intakeID1, const int intakeID2, const int wristID);
}

/*void Arm::ClimbSM()
{
    switch(m_Elevator->m_ClimberState)
    {
        case Elevator::ST_EXTEND :
            // elevator extending
            if(m_Elevator->ElevatorAtTarget())
            {
                m_Elevator->RequestElevatorPosition(CONSTANT("CLIMB_RETRACT"));
                m_Elevator->m_ClimberState = Elevator::ST_RETRACT;
            }
            break;
        case Elevator::ST_RETRACT :
            // elevator retracting
            if(m_Elevator->ElevatorAtTarget())
            {
                // end wrist lockout
            }
            break;
        case Elevator::ST_DEFAULT :
            m_Elevator->RequestElevatorPosition(CONSTANT("CLIMB_EXT"));
            m_Elevator->m_ClimberState = Elevator::ST_EXTEND;
            break;
        default :
            // do nothing
            break;
    }
   
}



void Arm::Handle()
{
    void Elevator::Handle();
    void Shooter::Handle();
}

*/