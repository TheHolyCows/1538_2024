#include "CowRobot.h"

CowRobot::CowRobot()
{
    m_MatchTime     = 0;
    m_StartTime     = 0;
    m_DSUpdateCount = 0;

    m_PowerDistributionPanel = new frc::PowerDistribution(1, frc::PowerDistribution::ModuleType::kRev);

    // mxp board was removed from robot - can remove this code
    m_LEDDisplay = nullptr;

    m_Gyro = CowPigeon::GetInstance();
    m_Accelerometer = new frc::BuiltInAccelerometer(frc::BuiltInAccelerometer::kRange_4G);

    // Set up drivetrain
    // TODO: reset constants needs to reset this
    // fl, fr, bl, br
    // drive motor, angle motor, encoder canId's
    SwerveDrive::ModuleConstants swerveModuleConstants[4]{
        SwerveDrive::ModuleConstants{ 1, 2, 25, CONSTANT("SWERVE_FL_ENCODER_OFFSET") },
        SwerveDrive::ModuleConstants{ 3, 4, 26, CONSTANT("SWERVE_FR_ENCODER_OFFSET") },
        SwerveDrive::ModuleConstants{ 5, 6, 27, CONSTANT("SWERVE_BL_ENCODER_OFFSET") },
        SwerveDrive::ModuleConstants{ 7, 8, 28, CONSTANT("SWERVE_BR_ENCODER_OFFSET") }
    };

    m_Drivetrain = new SwerveDrive(swerveModuleConstants, CONSTANT("WHEEL_BASE"));
    m_DriveController = new SwerveDriveController(*m_Drivetrain);

    // m_Shooter = new Shooter(11, 12, 9, 10, 13);

    ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(100_Hz, GetSynchronizedSignals());
}

std::vector<ctre::phoenix6::BaseStatusSignal*> CowRobot::GetSynchronizedSignals()
{
    std::vector<ctre::phoenix6::BaseStatusSignal*> signals;
    std::vector<ctre::phoenix6::BaseStatusSignal*> drivetrainSignals = m_Drivetrain->GetSynchronizedSignals();
    std::vector<ctre::phoenix6::BaseStatusSignal*> gyroSignals = m_Gyro->GetSynchronizedSignals();
    // std::vector<ctre::phoenix6::BaseStatusSignal*> shooterSignals = m_Shooter->GetSynchronizedSignals();
    signals.insert(signals.end(), drivetrainSignals.begin(), drivetrainSignals.end());
    signals.insert(signals.end(), gyroSignals.begin(), gyroSignals.end());
    // signals.insert(signals.end(), shooterSignals.begin(), shooterSignals.end());

    return signals;
}

/**
 * @brief reset drivetrain encoders and gyro
 */
void CowRobot::Reset()
{
    m_MatchTime = 0;

    m_Drivetrain->ResetConstants();
    m_DriveController->ResetConstants();
    // m_Shooter->ResetConstants();
    // m_Controller->ResetConstants(); TODO: error

    // Vision::GetInstance()->Reset();

    CowLib::CowLogger::GetInstance()->Reset();
}

/**
 * @brief
 *
 * @param controller
 */
void CowRobot::SetController(GenericController *controller)
{
    m_Controller = controller;
}

void CowRobot::PrintToDS()
{
    if (m_DSUpdateCount++ % 10 == 0)
    {
        m_DSUpdateCount = 1;
    }
}

// Used to handle the recurring logic funtions inside the robot.
// Please call this once per update cycle.
void CowRobot::Handle()
{
    m_MatchTime = CowLib::CowTimer::GetFPGATimestamp() - m_StartTime;

    if (m_Controller == nullptr)
    {
        printf("No controller for CowRobot!!\n");
        return;
    }

    // Synchronize and sample time-critical sensors
    ctre::phoenix6::BaseStatusSignal::WaitForAll(0_ms, GetSynchronizedSignals());

    m_Controller->Handle(this);
    m_Drivetrain->Handle();
    // m_Shooter->Handle();

    // logger code below should have checks for debug mode before sending out data
    CowLib::CowLogger::GetInstance()->Handle();
    // log the following every 200 ms
    // if (m_DSUpdateCount % 15 == 0)
    // {
    //     // m_DSUpdateCount is reset in PrintToDS
    //     CowLib::CowLogger::LogGyro(m_Gyro);
    //     CowLib::CowLogger::LogPose(m_Drivetrain->GetPoseX(), m_Drivetrain->GetPoseY(), m_Drivetrain->GetPoseRot());
    // }

    //    // APRIL TAG BOTPOSE
    //    std::optional<Vision::BotPoseResult> visionPose = Vision::GetInstance()->GetBotPose();
    //    if (visionPose.has_value())
    //    {
    //        m_Drivetrain->AddVisionMeasurement((*visionPose).pose, (*visionPose).timestamp);
    //    }

    // accelerometers
    // double zVal = m_ZFilter.Calculate(m_Accelerometer->GetZ());

    // positive is true, negative is false
    // bool direction = (zVal - m_PrevZ) > 0 ? true : false;
    // m_PrevZ = zVal;

    PrintToDS();

    // double xAccel = m_Accelerometer->GetX();
    // // frc::SmartDashboard::PutNumber("x accel", xAccel);
    // // frc::SmartDashboard::PutNumber("gyro yaw", m_Gyro->GetYawDegrees());
    // if (m_Arm->GetArmState() == ARM_HUMAN && m_Arm->GetClawState() == CLAW_INTAKE
    //     && fabs(xAccel) > CONSTANT("GYRO_RESET_ACCEL"))
    // {
    //     m_Gyro->SetYaw(0);
    //     m_DriveController->ResetHeadingLock();
    // }
}

void CowRobot::StartTime()
{
    m_StartTime = CowLib::CowTimer::GetFPGATimestamp();
}

void CowRobot::DoNothing()
{
    // TODO: make the robot stop (including drive)
}


// void CowRobot::ClimbSM()
// {
//     switch(m_Elevator->m_ClimberState)
//     {
//         case Elevator::ST_EXTEND :
//             // elevator extending
//             if(m_Elevator->ElevatorAtTarget())
//             {
//                 m_Elevator->RequestElevatorPosition(CONSTANT("CLIMB_RETRACT"));
//                 m_Elevator->m_ClimberState = Elevator::ST_RETRACT;
//             }
//             break;
//         case Elevator::ST_RETRACT :
//             // elevator retracting
//             if(m_Elevator->ElevatorAtTarget())
//             {
//                 // end wrist lockout
//             }
//             break;
//         case Elevator::ST_DEFAULT :
//             m_Elevator->RequestElevatorPosition(CONSTANT("CLIMB_EXT"));
//             m_Elevator->m_ClimberState = Elevator::ST_EXTEND;
//             break;
//         default :
//             // do nothing
//             break;
//     }
// }