#include "OperatorController.h"

OperatorController::OperatorController(GenericControlBoard *controlboard)
    : m_CB(controlboard),
      m_LastShotDistance(0.0),
      m_LastShotPivot(0.0),
      m_LastShotWrist(0.0)
{
    m_TrackingCooldownTimer = 0.0;
    m_ClimberLatch = false;
}

void OperatorController::Handle(CowRobot *bot)
{
    // LED
    Vision::LEDState ledState = Vision::LEDState::OFF;

    if (bot->m_Shooter->GetIntakeState() == Shooter::IntakeState::DETECT_HOLD)
    {
        ledState = Vision::LEDState::HOLD;
    }
    else if (bot->m_Shooter->GetIntakeState() == Shooter::IntakeState::DETECT_ACTIVE)
    {
        ledState = Vision::LEDState::INTAKING;
    }
    else if (bot->m_Shooter->GetIntakeState() == Shooter::IntakeState::EXHAUST)
    {
        ledState = Vision::LEDState::EXHAUSTING;
    }

    // if (m_CB->GetDriveAxis(2) > 0.8 && m_CB->GetDriveAxis(6) > 0.8)
    // {
    //     bot->GetDrivetrain()->SetLocked(true);
    //     bot->GetDrivetrain()->SetVelocity(0, 0, 0);
    // }
    // else
    // {
    //     bot->GetDrivetrain()->SetLocked(false);
    // }

    if (m_CB->GetVisionTargetButton())
    {
        // Drivetrain targetting
        frc::Translation2d targetXY = bot->m_Vision->GetTargetXY(bot->m_Alliance);

        SwerveDriveController::DriveLookAtRequest req = {
            .inputX = m_CB->GetLeftDriveStickY(),
            .inputY = -m_CB->GetLeftDriveStickX(),
            .targetX = units::foot_t(targetXY.X()).value(), //CONSTANT("GOAL_X") - CONSTANT("GOAL_X_OFFSET"),
            .targetY = units::foot_t(targetXY.Y()).value(), //CONSTANT("GOAL_Y") - CONSTANT("GOAL_Y_OFFSET"),
            .robotSide = SwerveDriveController::RobotSide::BACK,
            .lookaheadTime = CONSTANT("POSE_LOOKAHEAD_TIME")
        };

        bot->GetDriveController()->Request(req);

        double wristBias = m_CB->GetBiasSwitch() * CONSTANT("WRIST_BIAS_STEP");
        frc::Pose2d lookaheadPose = bot->GetDrivetrain()->Odometry()->Lookahead(CONSTANT("POSE_LOOKAHEAD_TIME"))
                                                                    .value_or(bot->GetDrivetrain()->GetPose());
        double dist = bot->m_Vision->GetTargetDist(bot->m_Alliance, lookaheadPose);

        double wristSetpoint = (CONSTANT("WRIST_AUTO_RANGING_A") * std::pow(dist, 3)) +
                               (CONSTANT("WRIST_AUTO_RANGING_B") * std::pow(dist, 2)) +
                               (CONSTANT("WRIST_AUTO_RANGING_C") * std::pow(dist, 1)) +
                               (CONSTANT("WRIST_AUTO_RANGING_D") * std::pow(dist, 0));

        wristSetpoint += CONSTANT("WRIST_STATIC_BIAS");
        wristSetpoint += wristBias;

        if (m_CB->GetOperatorButton(SWITCH_HI_LO))
        {
            bot->m_Pivot->SetTargetAngle(CONSTANT("PIVOT_AUTORANGING_SETPOINT_LO"));
        }
        else
        {
            bot->m_Pivot->SetTargetAngle(CONSTANT("PIVOT_AUTORANGING_SETPOINT_HI"));
        }

        bot->m_Wrist->SetAngle(wristSetpoint, bot->m_Pivot->GetAngle());

        // Shooter
        if (m_CB->GetOperatorButton(SWITCH_SHOOTER))
        {
            bot->m_Shooter->PrimeShooter(bot->m_ShooterRangeMap[dist]);
        }

        // printf("dist: %f | wrist: %f | knob: %f\n", dist, wristSetpoint, wristBias);

        // LED
        if (bot->m_Wrist->GetSetpoint() >= CONSTANT("WRIST_MAX_ANGLE"))
        {
            ledState = Vision::LEDState::IMPOSSIBLE_SHOT;
        }
        else if (dist < CONSTANT("SHOOTING_THRESHOLD_DISTANCE") &&
            bot->GetDriveController()->IsOnTarget() &&
            bot->m_Shooter->IsReady())
        {
            ledState = Vision::LEDState::ON_TARGET;
        }

        // Record data from previous shot
        if (m_CB->GetOperatorButton(BUTTON_SHOOT))
        {
            m_LastShotDistance = dist;
            m_LastShotPivot = bot->m_Pivot->GetTargetAngle();
            m_LastShotWrist = wristSetpoint;
        }
    }
    else if (m_CB->GetVisionTargetPassButton())
    {
        // Drivetrain targetting
        std::tuple<frc::Translation2d, double> targetTuple = bot->m_Vision->GetPassTargetXY(bot->m_Alliance, bot->GetDrivetrain()->GetPose());

        frc::Translation2d targetXY = std::get<0>(targetTuple);
        double shooterPower = std::get<1>(targetTuple);

        SwerveDriveController::DriveLookAtRequest req = {
            .inputX = m_CB->GetLeftDriveStickY(),
            .inputY = -m_CB->GetLeftDriveStickX(),
            .targetX = units::foot_t(targetXY.X()).value(),
            .targetY = units::foot_t(targetXY.Y()).value(),
            .robotSide = SwerveDriveController::RobotSide::BACK,
            .lookaheadTime = 0.0
        };

        bot->GetDriveController()->Request(req);

        double wristBias = m_CB->GetBiasSwitch() * CONSTANT("WRIST_BIAS_STEP");
        double dist = bot->m_Vision->GetTargetDist(bot->m_Alliance, bot->GetDrivetrain()->GetPose());

        bot->m_Pivot->SetTargetAngle(CONSTANT("PASS_PIVOT"));
        bot->m_Wrist->SetAngle(CONSTANT("PASS_WRIST"), bot->m_Pivot->GetTargetAngle());

        // LED
        if (bot->m_Shooter->IsReady())
        {
            ledState = Vision::LEDState::ON_TARGET;
        }

        // Shooter
        bot->m_Shooter->PrimeShooter(shooterPower);
    }
    else if (m_CB->GetDriveAxis(3) > 0.8) // Align heading
    {
        SwerveDriveController::DriveLockHeadingRequest req = {
            .inputX = m_CB->GetLeftDriveStickY(),
            .inputY = -m_CB->GetLeftDriveStickX()
        };

        bot->GetDriveController()->Request(req);
    }
    else
    {
        // standard drive with field/bot relative option

        // From the driver station perspective, +x is right, +y is away from
        // the driver station, and +rotation is a counter clockwise rotation

        // TODO: Flip depending on alliance color
        SwerveDriveController::DriveManualRequest req = {
            .inputX = m_CB->GetLeftDriveStickY(),
            .inputY = -m_CB->GetLeftDriveStickX(),
            .inputRotation = -m_CB->GetRightDriveStickX()
        };

        bot->GetDriveController()->Request(req);
    }
    if (m_CB->GetOperatorButton(BUTTON_AMP))
    {

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

    if (m_CB->GetOperatorButton(SWITCH_SHOOTER))
    {
        if (!m_CB->GetVisionTargetButton() && !m_CB->GetVisionTargetPassButton())
        {
            bot->m_Shooter->PrimeShooter(CONSTANT("SHOOTER_RANGE_VALUE_1"));
        }
    }
    else if(m_CB->GetOperatorButton(BUTTON_EXHAUST))
    {
        bot->m_Shooter->Exhaust();
    }
    else if (!m_CB->GetVisionTargetPassButton())
    {
        bot->m_Shooter->StopShooter();
    }

    if (!m_CB->GetOperatorButton(SWITCH_CLIMB))
    {
        if (!m_ClimberLatch)
        {
            m_ClimberLatch = true;
            bot->m_Pivot->SetTargetAngle(CONSTANT("PIVOT_CLIMB_ANGLE"));
            bot->m_Wrist->SetAngle(CONSTANT("WRIST_CLIMB_ANGLE"), bot->m_Pivot->GetTargetAngle());
            bot->m_Elevator->SetExtension(CONSTANT("ELEVATOR_CLIMB_UP"), false);
        }
        else if (m_CB->GetOperatorButton(BUTTON_CLIMB))
        {
            bot->m_Pivot->SetTargetAngle(CONSTANT("PIVOT_CLIMB_ANGLE"));
            bot->m_Wrist->SetAngle(CONSTANT("WRIST_CLIMB_ANGLE"), bot->m_Pivot->GetTargetAngle());
            bot->m_Elevator->SetExtension(CONSTANT("ELEVATOR_CLIMB_DOWN"), true);
        }
        else if (m_CB->GetOperatorButton(BUTTON_HP))
        {
            bot->m_Pivot->SetTargetAngle(CONSTANT("PIVOT_HP_SETPOINT"));
            bot->m_Wrist->SetAngle(CONSTANT("WRIST_HP_SETPOINT"), bot->m_Pivot->GetTargetAngle());
            bot->m_Elevator->SetExtension(CONSTANT("ELEVATOR_HIGH"), false);
        }
    }
    else if (!m_CB->GetOperatorButton(SWITCH_HI_LO))
    {
        m_ClimberLatch = false;

        if (m_CB->GetOperatorButton(BUTTON_STOW))
        {
            bot->m_Pivot->SetTargetAngle(CONSTANT("PIVOT_STOW_SETPOINT"));
            bot->m_Wrist->SetAngle(CONSTANT("WRIST_STOW_SETPOINT"), bot->m_Pivot->GetTargetAngle());
            bot->m_Elevator->SetExtension(CONSTANT("ELEVATOR_STOW_SETPOINT"), false);
        } else if (m_CB->GetOperatorButton(BUTTON_GROUND))
        {
            bot->m_Pivot->SetTargetAngle(CONSTANT("PIVOT_GROUND_SETPOINT"));
            bot->m_Wrist->SetAngle(CONSTANT("WRIST_GROUND_SETPOINT"), bot->m_Pivot->GetTargetAngle());
            // this is low on purpose
            bot->m_Elevator->SetExtension(CONSTANT("ELEVATOR_LOW"), false);
        }
        else if (m_CB->GetOperatorButton(BUTTON_LAUNCH))
        {
            bot->m_Pivot->SetTargetAngle(CONSTANT("PIVOT_LAUNCH_SETPOINT"));
            bot->m_Wrist->SetAngle(CONSTANT("WRIST_LAUNCH_SETPOINT"), bot->m_Pivot->GetTargetAngle());
            bot->m_Elevator->SetExtension(CONSTANT("ELEVATOR_HIGH"), false);
        }
        else if (m_CB->GetOperatorButton(BUTTON_HP))
        {
            bot->m_Pivot->SetTargetAngle(CONSTANT("PIVOT_HP_SETPOINT"));
            bot->m_Wrist->SetAngle(CONSTANT("WRIST_HP_SETPOINT"), bot->m_Pivot->GetTargetAngle());
            bot->m_Elevator->SetExtension(CONSTANT("ELEVATOR_HIGH"), false);
        }
        else if (m_CB->GetOperatorButton(BUTTON_AMP))
        {
            if (bot->m_Shooter->GetIntakeState() == Shooter::IntakeState::IDLE)
            {
                bot->m_Shooter->UpdateIntakeState(Shooter::IntakeState::DETECT_HOLD);
                bot->m_Shooter->UpdateIntakeDetectPosition(bot->m_Shooter->GetIntakePosition());
                bot->m_Shooter->UpdateIntakeMoveDistance(0.0);
            }

            if (bot->m_Pivot->GetAngle() > CONSTANT("PIVOT_AMP_MOVE_MIN_ANGLE"))
            {
                bot->m_Shooter->UpdateIntakeMoveDistance(CONSTANT("INTAKE_MOVE_DISTANCE_AMP"));
            }

            bot->m_Pivot->SetTargetAngle(CONSTANT("PIVOT_AMP_SETPOINT"));
            bot->m_Wrist->SetAngle(CONSTANT("WRIST_AMP_SETPOINT"), bot->m_Pivot->GetTargetAngle());
            bot->m_Elevator->SetExtension(CONSTANT("ELEVATOR_AMP_SETPOINT"), false);
        }
    }
    else // switch in low position and not climbing
    {
        m_ClimberLatch = false;
        if (m_CB->GetOperatorButton(BUTTON_STOW))
        {
            bot->m_Pivot->SetTargetAngle(CONSTANT("PIVOT_STOW_SETPOINT"));
            bot->m_Wrist->SetAngle(CONSTANT("WRIST_STOW_SETPOINT"), bot->m_Pivot->GetTargetAngle());
            bot->m_Elevator->SetExtension(CONSTANT("ELEVATOR_STOW_SETPOINT"), false);
        } else if (m_CB->GetOperatorButton(BUTTON_GROUND))
        {
            bot->m_Pivot->SetTargetAngle(CONSTANT("PIVOT_GROUND_SETPOINT"));
            bot->m_Wrist->SetAngle(CONSTANT("WRIST_GROUND_SETPOINT"), bot->m_Pivot->GetTargetAngle());
            bot->m_Elevator->SetExtension(CONSTANT("ELEVATOR_LOW"), false);
        }
        else if (m_CB->GetOperatorButton(BUTTON_LAUNCH))
        {
            bot->m_Pivot->SetTargetAngle(CONSTANT("PIVOT_LAUNCH_SETPOINT"));
            bot->m_Wrist->SetAngle(CONSTANT("WRIST_LAUNCH_SETPOINT"), bot->m_Pivot->GetTargetAngle());
            bot->m_Elevator->SetExtension(CONSTANT("ELEVATOR_LOW"), false);
        }
        else if (m_CB->GetOperatorButton(BUTTON_HP))
        {
            bot->m_Pivot->SetTargetAngle(CONSTANT("PIVOT_HP_SETPOINT"));
            bot->m_Wrist->SetAngle(CONSTANT("WRIST_HP_SETPOINT"), bot->m_Pivot->GetTargetAngle());
            bot->m_Elevator->SetExtension(CONSTANT("ELEVATOR_LOW"), false);
        }
        else if (m_CB->GetOperatorButton(BUTTON_AMP))
        {
            bot->m_Pivot->SetTargetAngle(CONSTANT("PIVOT_AMP_SETPOINT"));
            bot->m_Wrist->SetAngle(CONSTANT("WRIST_AMP_SETPOINT"), bot->m_Pivot->GetTargetAngle());
            bot->m_Elevator->SetExtension(CONSTANT("ELEVATOR_AMP_SETPOINT"), false);

            if (bot->m_Pivot->GetAngle() > CONSTANT("PIVOT_AMP_MOVE_MIN_ANGLE"))
            {
                bot->m_Shooter->UpdateIntakeMoveDistance(CONSTANT("INTAKE_MOVE_DISTANCE_AMP"));
            }
        }
    }

    if (m_CB->GetOperatorButton(BUTTON_HP))
    {
        printf("dist: %f, pivot: %f, wrist: %f\n", m_LastShotDistance, m_LastShotPivot, m_LastShotWrist);
    }

    bot->m_Vision->SetLEDState(ledState);

    // TODO: determine if this is the best way of doig this or to leave as is
    //    imo this adds needless complexity and would require break the "set() in disabled"
    //    functionality that we recently added
    //    however, we do need some aspect of this to update the wrist and elevator based on the
    //    current pivot angle
    // just gonna pass the pivot to handle of the two in question
    // bot->m_Pivot->SetAngle(m_PivotSetpoint);
    // bot->m_Wrist->SetAngle(m_WristSetpoint, bot->m_Pivot->GetAngle());
    // bot->m_Elevator->SetExtension(m_ElevatorSetpoint, bot->m_Pivot->GetAngle());

}
