#include "OperatorController.h"

OperatorController::OperatorController(GenericControlBoard *controlboard)
    : m_CB(controlboard)
{
    m_TrackingCooldownTimer = 0.0;
}

void OperatorController::Handle(CowRobot *bot)
{   
    // Vision::GetInstance()->SetInverted(inverted);

    if (m_CB->GetDriveAxis(2) > 0.8 && m_CB->GetDriveAxis(6) > 0.8)
    {
        bot->GetDrivetrain()->SetLocked(true);
        bot->GetDrivetrain()->SetVelocity(0, 0, 0);
    }
    else
    {
        bot->GetDrivetrain()->SetLocked(false);
    }

    if (m_CB->GetVisionTargetButton())
    {
        // Vision::GetInstance()->...
    }
    else if (m_CB->GetDriveAxis(3) > 0.8) // Align heading
    {
        bot->GetDriveController()->LockHeading(m_CB->GetLeftDriveStickY(), m_CB->GetLeftDriveStickX());
    }
    else
    {
        // standard drive with field/bot relative option
        
        // From the driver station perspective, +x is right, +y is away from
        // the driver station, and +rotation is a counter clockwise rotation
        bot->GetDriveController()->Drive(m_CB->GetLeftDriveStickY(),
                                         -m_CB->GetLeftDriveStickX(),
                                         -m_CB->GetRightDriveStickX(),
                                         true);
    }
   
    // if (m_CB->GetDriveButton(1))
    // {
    //     bot->m_Shooter->Intake();
    // }
    // else if(m_CB->GetDriveButton(2))
    // {
    //     bot->m_Shooter->Outtake();
    // }
    // else if (!m_CB->GetDriveButton(3))
    // {
    //     bot->m_Shooter->StopIntake();
    // }

    // if(m_CB->GetDriveButton(5))
    // {
    //     bot->m_Shooter->PrimeShooter();

    //     if(m_CB->GetDriveButton(3))
    //     {
    //         bot->m_Shooter->Shoot();
    //     }
    // }
    // else
    // {
    //     bot->m_Shooter->StopShooter();
    // }

    if (m_CB->GetOperatorButton(8))
    {
        bot->m_Pivot->SetAngle(CONSTANT("PIVOT_MID_SETPOINT"));
    }
    if (m_CB->GetOperatorButton(9))
    {
        bot->m_Pivot->SetAngle(CONSTANT("PIVOT_FAR_SETPOINT"));
    }

    if (m_CB->GetOperatorButton(5))
    {
        bot->m_Shooter->Intake();
    }
    else if (m_CB->GetOperatorButton(6))
    {
        bot->m_Shooter->Exhaust();
    }
    else
    {
        bot->m_Shooter->StopIntake();
    }

    if (m_CB->GetOperatorButton(4))
    {
        bot->m_Elevator->SetExtension(CONSTANT("ELEVATOR_HIGH"),bot->m_Pivot->GetSetpoint());
    }
    else
    {
        bot->m_Elevator->SetExtension(CONSTANT("ELEVATOR_LOW"),bot->m_Pivot->GetSetpoint());
    }

    if (m_CB->GetOperatorButton(2))
    {
        bot->m_Wrist->SetAngle(CONSTANT("WRIST_MIN_ANGLE"),bot->m_Pivot->GetSetpoint());
    }
    if (m_CB->GetOperatorButton(1))
    {
        bot->m_Wrist->SetAngle(CONSTANT("WRIST_MAX_ANGLE"),bot->m_Pivot->GetSetpoint());
    }
    
}
