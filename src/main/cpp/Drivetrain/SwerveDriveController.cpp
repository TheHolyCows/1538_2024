#include "SwerveDriveController.h"

SwerveDriveController::SwerveDriveController(SwerveDrive &drivetrain)
    : m_Drivetrain(drivetrain),
      m_Gyro(*CowPigeon::GetInstance()),
      m_HeadingLocked(false),
      m_VisionTargeting(false),
      m_TargetHeading(180)
{
    m_ExponentialFilter = std::make_unique<CowLib::CowExponentialFilter>(0);

    m_HeadingPIDController = std::make_unique<frc::ProfiledPIDController<units::degrees>>(0, 0, 0, frc::TrapezoidProfile<units::degrees>::Constraints());
    m_HeadingPIDController->EnableContinuousInput(units::degree_t{ 0 }, units::degree_t{ 360 });

    ResetConstants();
}

void SwerveDriveController::ResetConstants()
{
    m_ExponentialFilter->Reset(CONSTANT("STICK_EXPONENTIAL_MODIFIER"));

    m_HeadingPIDController->SetPID(CONSTANT("HEADING_P"), CONSTANT("HEADING_I"), CONSTANT("HEADING_D"));
    m_HeadingPIDController->SetConstraints(frc::TrapezoidProfile<units::degrees>::Constraints{
            units::degrees_per_second_t{ CONSTANT("HEADING_V") },
            units::degrees_per_second_squared_t{ CONSTANT("HEADING_A") } });
}

void SwerveDriveController::Drive(double x, double y, double rotation, bool fieldRelative)
{
    double centerOfRotationX = 0;
    double centerOfRotationY = 0;

    double omega = 0;

    // frc::SmartDashboard::PutNumber("rotation axis", rotation);

    // double heading = m_Gyro.GetYawDegrees();
    double heading = m_Drivetrain.GetPoseRot();

    if (fabs(rotation) > CONSTANT("STICK_DEADBAND"))
    {
        omega           = ProcessDriveAxis(rotation, CONSTANT("DESIRED_MIN_ANG_VEL"), CONSTANT("DESIRED_MAX_ANG_VEL"), false);
        m_HeadingLocked = false;
    }
    else
    {
        if (fabs(m_Gyro.GetYawVelocityDegrees()) < CONSTANT("HEADING_CAPTURE_THRESHOLD") && !m_HeadingLocked)
        {
            m_TargetHeading = heading;
            m_HeadingLocked = true;
        }
        else if (m_HeadingLocked)
        {
            omega = m_HeadingPIDController->Calculate(units::degree_t{ heading }, units::degree_t{ m_TargetHeading });
        }
    }

    // frc::SmartDashboard::PutNumber("heading locked", m_HeadingLocked);
    // frc::SmartDashboard::PutNumber("omega deg / sec", omega);

    double magnitude = sqrt(pow(x, 2.0) + pow(y, 2.0));

    if (magnitude > CONSTANT("STICK_DEADBAND"))
    {
        double magnitudeFiltered = ProcessDriveAxis(magnitude, CONSTANT("DESIRED_MIN_SPEED"), CONSTANT("DESIRED_MAX_SPEED"), false);
        double theta = atan2(y, x);

        x = cos(theta) * magnitudeFiltered;
        y = sin(theta) * magnitudeFiltered;
    }
    else
    {
        x = 0;
        y = 0;
    }

    if (x == 0 && y == 0 && fabs(rotation) < CONSTANT("STICK_DEADBAND") && !m_VisionTargeting)
    {
        omega = 0;
    }

    m_Drivetrain.SetVelocity(x, y, omega, fieldRelative, centerOfRotationX, centerOfRotationY);
}

void SwerveDriveController::DriveManual(double x, double y, double rotation)
{
    m_VisionTargeting = false;

    Drive(x, y, rotation, true);
}

void SwerveDriveController::DriveLookAt(double x, double y, double targetX, double targetY)
{
    m_TargetHeading = (atan2(m_Drivetrain.GetPoseY() - targetY, m_Drivetrain.GetPoseX() - targetX) / 3.1415) * 180;
    m_HeadingLocked = true;
    m_VisionTargeting = true;

    Drive(x, y, 0.0, true);
}

double SwerveDriveController::GetHeadingError()
{
    return m_HeadingPIDController->GetPositionError().value();
}

void SwerveDriveController::LockHeading(double x, double y, bool useRawInputs)
{
    double currentHeading = m_Gyro.GetYawDegrees();

    currentHeading = fmod(currentHeading, 360);
    if (currentHeading < 0)
    {
        currentHeading += 360;
    }

    if (currentHeading < 90 || currentHeading > 270)
    {
        m_TargetHeading = 0;
    }
    else
    {
        m_TargetHeading = 180;
    }

    m_HeadingLocked = true;

    // idk if this makes a difference but should ensure no weird PID to past heading things
    double omega = m_HeadingPIDController->Calculate(units::degree_t{ m_Gyro.GetYawDegrees() },
                                                     units::degree_t{ m_TargetHeading });

    if (!useRawInputs)
    {
        x = ProcessDriveAxis(x, CONSTANT("DESIRED_MIN_SPEED"), CONSTANT("DESIRED_MAX_SPEED"), false);
        y = ProcessDriveAxis(y, CONSTANT("DESIRED_MIN_SPEED"), CONSTANT("DESIRED_MAX_SPEED"), false);
    }

    m_Drivetrain.SetVelocity(x, y, omega, true, 0, 0);
}

// void SwerveDriveController::CubeAlign(double x)
// {
//     // Check if heading is aligned
//     if (fabs(fmod(m_Gyro.GetYawDegrees(), 180)) > CONSTANT("HEADING_TOLERANCE"))
//     {
//         // If not, run the heading lock function
//         LockHeading(x, 0);
//         return;
//     }

//     // Otherwise, continue with vision alignment

//     double y = Vision::GetInstance()->CubeYPID();

//     x = ProcessDriveAxis(x, CONSTANT("DESIRED_MAX_SPEED"), false);

//     m_Drivetrain.SetVelocity(x, y, 0, true, 0, 0, true);
// }

// void SwerveDriveController::ConeAlign(double x, double yInput)
// {
//     // Check if heading is aligned
//     if (fabs(fmod(m_Gyro.GetYawDegrees(), 180)) > CONSTANT("HEADING_TOLERANCE"))
//     {
//         // If not, run the heading lock function
//         LockHeading(x, yInput);
//         return;
//     }

//     // Otherwise, continue with vision alignment
//     double y = Vision::GetInstance()->ConeYPID();

//     // Override if yInput is above override threshold
//     if (fabs(yInput) < CONSTANT("CONE_Y_OVERRIDE_THRESHOLD"))
//     {
//         y = ProcessDriveAxis(yInput, CONSTANT("DESIRED_MAX_SPEED"), false);
//     }

//     x = ProcessDriveAxis(x, CONSTANT("DESIRED_MAX_SPEED"), false);

//     m_Drivetrain.SetVelocity(x, y, 0, true, 0, 0, true);
// }

void SwerveDriveController::ResetHeadingLock()
{
    m_HeadingLocked = false;
    // m_TargetHeading = m_Gyro.GetYawDegrees();
    m_TargetHeading = m_Drivetrain.GetPoseRot();
}

double SwerveDriveController::ProcessDriveAxis(double input, double scaleMin, double scaleMax, bool reverse)
{
    double inputDeadbanded = CowLib::Deadband(input, CONSTANT("STICK_DEADBAND"));

    // Offset the input by the deadband and then renormalize it
    if (inputDeadbanded > 0)
    {
        inputDeadbanded -= CONSTANT("STICK_DEADBAND");
    }
    else if (inputDeadbanded < 0)
    {
        inputDeadbanded += CONSTANT("STICK_DEADBAND");
    }

    inputDeadbanded = (inputDeadbanded / (1 - CONSTANT("STICK_DEADBAND")));

    if (inputDeadbanded == 0)
    {
        return 0;
    }
    else
    {
        // Exponential filter
        return ((m_ExponentialFilter->Filter(inputDeadbanded) * (scaleMax - scaleMin)) + (inputDeadbanded > 0 ? scaleMin : -scaleMin)) *
               (reverse ? -1 : 1);
    }
}
