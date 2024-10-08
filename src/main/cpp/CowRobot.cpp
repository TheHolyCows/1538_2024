#include "CowRobot.h"

int log_count = 0;

CowRobot::CowRobot()
{
    m_MatchTime     = 0;
    m_StartTime     = 0;
    m_DSUpdateCount = 0;

    // mxp board was removed from robot - can remove this code
    m_LEDDisplay = nullptr;

    m_Gyro = CowPigeon::GetInstance();
    m_Accelerometer = new frc::BuiltInAccelerometer(frc::BuiltInAccelerometer::kRange_4G);

    // m_LoadManager = new LoadManager();

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
    // lilly wuz here
    m_Pivot = new Pivot(10, 9, 29, CONSTANT("PIVOT_ENCODER_OFFSET"));
    m_Elevator = new Elevator(11, 12);
    m_Wrist = new Wrist(13, 30, CONSTANT("WRIST_ENCODER_OFFSET"));
    m_Vision = new Vision();
    m_Shooter = new Shooter(15, 16, 14, 50, m_Vision);
    // m_Fan = new Fan(51);

    // CowDrive signals
    std::vector<ctre::phoenix6::BaseStatusSignal*> drivetrainSignals = m_Drivetrain->GetSynchronizedSignals();
    std::vector<ctre::phoenix6::BaseStatusSignal*> pivotSignals = m_Pivot->GetSynchronizedSignals();
    std::vector<ctre::phoenix6::BaseStatusSignal*> gyroSignals = m_Gyro->GetSynchronizedSignals();
    m_CowDriveSignals.insert(m_CowDriveSignals.end(), drivetrainSignals.begin(), drivetrainSignals.end());
    m_CowDriveSignals.insert(m_CowDriveSignals.end(), pivotSignals.begin(), pivotSignals.end());
    m_CowDriveSignals.insert(m_CowDriveSignals.end(), gyroSignals.begin(), gyroSignals.end());

    // CowBus signals
    std::vector<ctre::phoenix6::BaseStatusSignal*> shooterSignals = m_Shooter->GetSynchronizedSignals();
    std::vector<ctre::phoenix6::BaseStatusSignal*> elevatorSignals = m_Elevator->GetSynchronizedSignals();
    std::vector<ctre::phoenix6::BaseStatusSignal*> wristSignals = m_Wrist->GetSynchronizedSignals();
    m_CowBusSignals.insert(m_CowBusSignals.end(), shooterSignals.begin(), shooterSignals.end());
    m_CowBusSignals.insert(m_CowBusSignals.end(), elevatorSignals.begin(), elevatorSignals.end());
    m_CowBusSignals.insert(m_CowBusSignals.end(), wristSignals.begin(), wristSignals.end());

    // Set update frequency for signals
    ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(200_Hz, m_CowDriveSignals);
    ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(200_Hz, m_CowBusSignals);
}

/**
 * @brief reset drivetrain encoders and gyro
 */
void CowRobot::Reset()
{
    m_MatchTime = 0;

    m_Drivetrain->Reset();
    m_DriveController->ResetConstants();

    m_Pivot->ResetConstants();
    m_Elevator->ResetConstants();
    m_Wrist->ResetConstants();
    m_Shooter->ResetConstants();
    m_PivotRangeMap.clear();
    m_ShooterRangeMap.clear();

    m_PivotRangeMap.insert(CONSTANT("PIVOT_RANGE_DIST_1"), CONSTANT("PIVOT_RANGE_VALUE_1"));
    m_PivotRangeMap.insert(CONSTANT("PIVOT_RANGE_DIST_2"), CONSTANT("PIVOT_RANGE_VALUE_2"));
    m_PivotRangeMap.insert(CONSTANT("PIVOT_RANGE_DIST_3"), CONSTANT("PIVOT_RANGE_VALUE_3"));
    m_PivotRangeMap.insert(CONSTANT("PIVOT_RANGE_DIST_4"), CONSTANT("PIVOT_RANGE_VALUE_4"));
    m_PivotRangeMap.insert(CONSTANT("PIVOT_RANGE_DIST_5"), CONSTANT("PIVOT_RANGE_VALUE_5"));

    m_ShooterRangeMap.insert(CONSTANT("SHOOTER_RANGE_DIST_1"), CONSTANT("SHOOTER_RANGE_VALUE_1"));
    m_ShooterRangeMap.insert(CONSTANT("SHOOTER_RANGE_DIST_2"), CONSTANT("SHOOTER_RANGE_VALUE_2"));

    // m_Controller->ResetConstants(); TODO: error

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
    if (m_DSUpdateCount++ % 20 == 0)
    {
        m_DSUpdateCount = 1;
    }
}

void CowRobot::SampleSensors()
{
    // Synchronize and sample time-critical sensors
    ctre::phoenix6::BaseStatusSignal::RefreshAll(m_CowDriveSignals);
    ctre::phoenix6::BaseStatusSignal::RefreshAll(m_CowBusSignals);

    // Sample sensors
    m_Vision->SampleSensors();

    // Feed vision sample into pose estimator
    std::vector<Vision::Sample> samples = m_Vision->GetRobotPose();

    for (const Vision::Sample& sample : samples)
    {
        m_Drivetrain->AddVisionMeasurement(sample);
    }

    m_Drivetrain->SampleSensors();

    // Load Manager
    // m_LoadManager->Handle();

    if (log_count == 0)
    {
        // printf("WATT-HOURS CONSUMED: %f, INSTANTANEOUS LOAD %f, LIMIT %f\n", (m_LoadManager->GetEnergyConsumed() / 3600_s).value(), m_LoadManager->GetInstantaneousLoad().value(), m_LoadManager->GetSwerveDriveBudget().value());
    }

    log_count = (log_count + 1) % 20;
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

    m_Drivetrain->SetCurrentLimit(200.0_A);

    m_Controller->Handle(this);
    m_Drivetrain->Handle();
    m_DriveController->Handle();

    m_Pivot->Handle(m_Elevator->GetPosition());
    m_Elevator->Handle(m_Pivot);
    m_Wrist->Handle(m_Pivot);
    m_Shooter->Handle();
    // m_Fan->Handle();

    // // logger code below should have checks for debug mode before sending out data
    CowLib::CowLogger::GetInstance()->Handle();
    // log the following every 200 ms
    if (m_DSUpdateCount % 20 == 0)
    {
        // m_DSUpdateCount is reset in PrintToDS
        CowLib::CowLogger::LogGyro(m_Gyro);
        CowLib::CowLogger::LogPose(m_Drivetrain->GetPoseX(), m_Drivetrain->GetPoseY(), m_Drivetrain->GetPoseRot());
    }

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