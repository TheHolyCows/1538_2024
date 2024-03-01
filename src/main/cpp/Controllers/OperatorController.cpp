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

    if (m_CB->GetDriveAxis(5) > 0.8 || m_CB->GetDriveAxis(6) > 0.8)
    {
        bot->GetDriveController()->LockHeading(m_CB->GetLeftDriveStickY(), m_CB->GetLeftDriveStickX());
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

        // TODO: Flip depending on alliance color
        bot->GetDriveController()->Drive(-m_CB->GetLeftDriveStickY(),
                                         m_CB->GetLeftDriveStickX(),
                                         -m_CB->GetRightDriveStickX(),
                                         true);
    }
    // intake calibration - remove in PROD
    if (m_CB->GetOperatorButton(BUTTON_AMP))
    {
        bot->m_Shooter->CalibrateIntake();
    }
    else if (m_CB->GetOperatorButton(BUTTON_INTAKE))
    {
        bot->m_Shooter->Intake();
    }
    else if(m_CB->GetOperatorButton(BUTTON_SHOOT))
    {
        bot->m_Shooter->Shoot();
    }
    else if(m_CB->GetOperatorButton(BUTTON_EXHAUST))
    {
        bot->m_Shooter->Exhaust();
    }
    else
    {
        bot->m_Shooter->StopIntake();
    }

    if (!m_CB->GetOperatorButton(SWITCH_SHOOTER))
    {
        bot->m_Shooter->PrimeShooter();
    }
    else
    {
        bot->m_Shooter->StopShooter();
    }

    if (m_CB->GetOperatorButton(SWITCH_CLIMB))
    {
        bot->m_Pivot->SetAngle(CONSTANT("PIVOT_CLIMB_ANGLE"));
        bot->m_Elevator->SetExtension(CONSTANT("ELEVATOR_CLIMB_UP"),bot->m_Pivot->GetSetpoint());
    }
    else if (m_CB->GetOperatorButton(SWITCH_HI_LO))
    {
        if (m_CB->GetOperatorButton(BUTTON_SETPOINT_1))
        {
            bot->m_Pivot->SetAngle(CONSTANT("PIVOT_NEAR_SETPOINT"));
            bot->m_Wrist->SetAngle(CONSTANT("WRIST_NEAR_SETPOINT"), bot->m_Pivot->GetSetpoint());
            bot->m_Elevator->SetExtension(CONSTANT("ELEVATOR_HIGH"),bot->m_Pivot->GetSetpoint());
        }
        else if (m_CB->GetOperatorButton(BUTTON_SETPOINT_2))
        {
            bot->m_Pivot->SetAngle(CONSTANT("PIVOT_MID_SETPOINT"));
            bot->m_Wrist->SetAngle(CONSTANT("WRIST_MID_SETPOINT"), bot->m_Pivot->GetSetpoint());
            bot->m_Elevator->SetExtension(CONSTANT("ELEVATOR_HIGH"),bot->m_Pivot->GetSetpoint());
        }
        else if (m_CB->GetOperatorButton(BUTTON_SETPOINT_3))
        {
            bot->m_Pivot->SetAngle(CONSTANT("PIVOT_FAR_SETPOINT"));
            bot->m_Wrist->SetAngle(CONSTANT("WRIST_FAR_SETPOINT"), bot->m_Pivot->GetSetpoint());
            bot->m_Elevator->SetExtension(CONSTANT("ELEVATOR_HIGH"),bot->m_Pivot->GetSetpoint());
        }
    }
    else // switch in low position and not climbing
    {
        if (m_CB->GetOperatorButton(BUTTON_SETPOINT_1))
        {
            bot->m_Pivot->SetAngle(CONSTANT("PIVOT_NEAR_SETPOINT"));
            bot->m_Wrist->SetAngle(CONSTANT("WRIST_NEAR_SETPOINT"), bot->m_Pivot->GetSetpoint());
            bot->m_Elevator->SetExtension(CONSTANT("ELEVATOR_LOW"),bot->m_Pivot->GetSetpoint());
        }
        else if (m_CB->GetOperatorButton(BUTTON_SETPOINT_2))
        {
            bot->m_Pivot->SetAngle(CONSTANT("PIVOT_MID_SETPOINT"));
            bot->m_Wrist->SetAngle(CONSTANT("WRIST_MID_SETPOINT"), bot->m_Pivot->GetSetpoint());
            bot->m_Elevator->SetExtension(CONSTANT("ELEVATOR_LOW"),bot->m_Pivot->GetSetpoint());
        }
        else if (m_CB->GetOperatorButton(BUTTON_SETPOINT_3))
        {
            bot->m_Pivot->SetAngle(CONSTANT("PIVOT_FAR_SETPOINT"));
            bot->m_Wrist->SetAngle(CONSTANT("WRIST_FAR_SETPOINT"), bot->m_Pivot->GetSetpoint());
            bot->m_Elevator->SetExtension(CONSTANT("ELEVATOR_LOW"),bot->m_Pivot->GetSetpoint());
        }
    }

    // if (m_CB->GetOperatorButton(4))
    // {
    //     bot->m_Elevator->SetExtension(CONSTANT("ELEVATOR_HIGH"),bot->m_Pivot->GetSetpoint());
    // }
    // else
    // {
    //     bot->m_Elevator->SetExtension(CONSTANT("ELEVATOR_LOW"),bot->m_Pivot->GetSetpoint());
    // }

    // if (m_CB->GetOperatorButton(2))
    // {
    //     bot->m_Wrist->SetAngle(CONSTANT("WRIST_MIN_ANGLE"),bot->m_Pivot->GetSetpoint());
    // }
    // if (m_CB->GetOperatorButton(1))
    // {
    //     bot->m_Wrist->SetAngle(CONSTANT("WRIST_MAX_ANGLE"),bot->m_Pivot->GetSetpoint());
    // }
    
}
