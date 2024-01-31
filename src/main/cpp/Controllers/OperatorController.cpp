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
        bot->GetDriveController()->Drive(m_CB->GetLeftDriveStickY(),
                                         m_CB->GetLeftDriveStickX(),
                                         m_CB->GetRightDriveStickX() * -1,
                                         true);
    }
   
    if (m_CB->GetDriveButton(1))
    {
        bot->m_Shooter->Intake();
    }
    else if(m_CB->GetDriveButton(3))
    {
        bot->m_Shooter->Outtake();
    }
    else
    {
        bot->m_Shooter->StopIntake();
    }

    if(m_CB->GetDriveButton(2))
    {
        bot->m_Shooter->PrimeShooter();
    }
    else if(m_CB->GetDriveButton(4))
    {
        bot->m_Shooter->Shoot();
    }

    // Do we need an else state here to return ShooterState to IDLE?
    
}
