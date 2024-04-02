#include "SwerveDrive.h"

#include <iostream>

/**
 * @brief Construct a new SwerveDrive object
 *
 * @param moduleConstants Array of constants for each module
 */
SwerveDrive::SwerveDrive(ModuleConstants moduleConstants[4], double wheelBase)
    : m_Gyro(CowPigeon::GetInstance()),
      m_Kinematics(new CowLib::CowSwerveKinematics(wheelBase)),
      m_Odometry(std::make_shared<CowLib::CowSwerveOdometry>(m_Kinematics, 0, 0, 0, 0, CONSTANT("POSE_BUFFER_SIZE"))),
      m_Pose({0_m, 0_m, 0_deg}),
      m_PrevChassisSpeeds({ 0.0_mps, 0.0_mps, units::radians_per_second_t(0) }),
      m_Locked(false)
{
    if (!frc::RobotBase::IsReal())
    {
        for (int i = 0; i < 4; i++)
        {
            m_Modules[i] = new SwerveModuleSim(i, 8, 500, 0, moduleConstants[i].encoderOffset);
        }
    }
    else
    {
        for (int i = 0; i < 4; i++)
        {
            m_Modules[i] = new SwerveModule(i,
                                            moduleConstants[i].driveMotorId,
                                            moduleConstants[i].rotationMotorId,
                                            moduleConstants[i].encoderId,
                                            moduleConstants[i].encoderOffset);
        }
    }

    ResetOdometry(frc::Pose2d(50_ft, 20_ft, 180_deg));
    Reset();
}

SwerveDrive::~SwerveDrive()
{
    delete m_Kinematics;

    for (auto module : m_Modules)
    {
        delete module;
    }
}

std::vector<ctre::phoenix6::BaseStatusSignal*> SwerveDrive::GetSynchronizedSignals()
{
    std::vector<ctre::phoenix6::BaseStatusSignal*> signals;

    for (auto module : m_Modules)
    {
        std::vector<ctre::phoenix6::BaseStatusSignal*> moduleSignals = module->GetSynchronizedSignals();
        signals.insert(signals.end(), moduleSignals.begin(), moduleSignals.end());
    }

    return signals;
}

std::shared_ptr<CowLib::CowSwerveOdometry> SwerveDrive::Odometry()
{
    return m_Odometry;
}

/**
 * @brief Sets drive velocity
 *
 * @param x Translational X velocity in feet per second
 * @param y Translational Y velocity in feet per second
 * @param rotation Rotational velocity in degrees per second
 * @param isFieldRelative Controls whether drive is field relative, default true
 * @param centerOfRotationX X component of center of rotation
 * @param centerOfRotationY Y component of center of rotation
 */
void SwerveDrive::SetVelocity(double vx,
                              double vy,
                              double omega,
                              bool isFieldRelative,
                              double centerOfRotationX,
                              double centerOfRotationY,
                              bool force)
{
    CowLib::CowChassisSpeeds chassisSpeeds{};

    if (isFieldRelative)
    {
        chassisSpeeds = CowLib::CowChassisSpeeds::FromFieldRelativeSpeeds(vx, vy, omega, m_Gyro->GetYawDegrees());
        // chassisSpeeds = CowLib::CowChassisSpeeds::FromFieldRelativeSpeeds(vx, vy, omega, GetPoseRot());

        // save off current chassis speeds for auto mode - this is always robot relative
        m_PrevChassisSpeeds.vx = units::feet_per_second_t(vx);
        m_PrevChassisSpeeds.vy = units::feet_per_second_t(vy);
        m_PrevChassisSpeeds.omega = units::degrees_per_second_t(omega);
    }
    else
    {
        chassisSpeeds = CowLib::CowChassisSpeeds{ vx, vy, omega };

        // save off current chassis speeds for auto mode
        m_PrevChassisSpeeds.vx = units::feet_per_second_t(vx);
        m_PrevChassisSpeeds.vy = units::feet_per_second_t(vy);
        m_PrevChassisSpeeds.omega = units::degrees_per_second_t(omega);
    }

    std::array<CowLib::CowSwerveModuleState, 4> moduleStates;

    if (m_Locked)
    {
        // hardcoded angles for e-brake X formation
        double angles[4] = { 45, 315, 135, 225 };

        for (int i = 0; i < 4; i++)
        {
            moduleStates[i] = CowLib::CowSwerveModuleState{ 0.0, angles[i] };
        }
    }
    else
    {
        moduleStates = m_Kinematics->CalculateModuleStates(chassisSpeeds, centerOfRotationX, centerOfRotationY);
    }

    // Scale module speeds
    CowLib::CowSwerveKinematics::DesaturateSpeeds(&moduleStates, CONSTANT("SWERVE_MAX_SPEED"));

    // Set module target state
    for (auto module : m_Modules)
    {
        module->SetTargetState(moduleStates[module->GetID()], m_Locked);
    }
}

/**
 * @brief Same as the other SetVelocity, but using CowChassisSpeeds used in autonomous
 * @param chassisSpeeds CowChassisSpeeds struct
 * @param isFieldRelative
 * @param centerOfRotationX X component of center of rotation
 * @param centerOfRotationY Y component of center of rotation
 */
void SwerveDrive::SetVelocity(CowLib::CowChassisSpeeds chassisSpeeds,
                              bool isFieldRelative,
                              double centerOfRotationX,
                              double centerOfRotationY,
                              bool force)
{
    SetVelocity(chassisSpeeds.vx,
                chassisSpeeds.vy,
                chassisSpeeds.omega,
                isFieldRelative,
                centerOfRotationX,
                centerOfRotationY,
                force);
}

frc::Pose2d SwerveDrive::GetPose()
{
    return m_Odometry->GetWPIPose();
}

frc::ChassisSpeeds SwerveDrive::GetChassisSpeeds() {
    return m_PrevChassisSpeeds;
}

/**
 * @brief Get the Pose X value in feet
 *
 * @return double
 */
double SwerveDrive::GetPoseX()
{
    return m_Pose.X().convert<units::foot>().value();
}

/**
 * @brief Get the Pose Y value in feet
 *
 * @return double
 */
double SwerveDrive::GetPoseY()
{
    return m_Pose.Y().convert<units::foot>().value();
}

/**
 * @brief Get the pose rotation value in degrees
 *
 * @return double
 */
double SwerveDrive::GetPoseRot()
{
    return m_Pose.Rotation().Degrees().value();
}

/**
 * @brief Returns current locked state
 *
 * @return Whether drive is locked
 */
bool SwerveDrive::GetLocked() const
{
    return m_Locked;
}

/**
 * @brief Sets locked state
 * moves wheels into X formation, braking in place on charge station
 * call to this within Operator Controller is all that is needed to e-brake
 * @param isLocked
 */
void SwerveDrive::SetLocked(bool isLocked)
{
    m_Locked = isLocked;
}

void SwerveDrive::SetBrakeMode(bool brakeMode)
{
    for (auto module : m_Modules)
    {
        module->SetBrakeMode(brakeMode);
    }
}

void SwerveDrive::AddVisionMeasurement(Vision::Sample sample)
{
    if (sample.tagCount == 0)
    {
        return;
    }

    frc::Pose2d visionPose = sample.pose3d.ToPose2d();
    double translationStdDev = CONSTANT("POSE_XY_STD_DEV_SCALE") / std::pow(sample.tagCount, CONSTANT("POSE_CNT_EXP"));
    double rotationStdDev = CONSTANT("POSE_ROT_STD_DEV_SCALE") / std::pow(sample.tagCount, CONSTANT("POSE_CNT_EXP"));

    if (sample.tagCount == 1)
    {
        // this distance is in meters, is that ok
        double distance = std::sqrt(std::pow(m_Pose.X().value() - visionPose.X().value(), 2) + std::pow(m_Pose.Y().value() - visionPose.Y().value(), 2));

        if (distance < CONSTANT("POSE_SINGLE_TAG_DIST"))
        {
            m_Odometry->GetInternalPoseEstimator()->AddVisionMeasurement(
            sample.pose3d.ToPose2d(),
            sample.timestamp,
            {translationStdDev, translationStdDev, rotationStdDev});
        }
    }
    else
    {
        m_Odometry->GetInternalPoseEstimator()->AddVisionMeasurement(
        sample.pose3d.ToPose2d(),
        sample.timestamp,
        {translationStdDev, translationStdDev, rotationStdDev});
    }

    m_PreviousVisionSample = sample;
}

void SwerveDrive::ResetConstants()
{
    for (auto module : m_Modules)
    {
        module->ResetConstants();
    }
}

/// @brief Resets encoders
void SwerveDrive::ResetEncoders()
{
    for (auto module : m_Modules)
    {
        module->ResetEncoders();
    }
}

void SwerveDrive::ResetOdometry(frc::Pose2d pose)
{
    ctre::phoenix6::BaseStatusSignal::WaitForAll(0_ms, GetSynchronizedSignals());
    ctre::phoenix6::BaseStatusSignal::WaitForAll(0_ms, m_Gyro->GetSynchronizedSignals());

    std::array<CowLib::CowSwerveModulePosition, 4> modulePositions{};
    for (auto module : m_Modules)
    {
        modulePositions[module->GetID()] = module->GetPosition();
    }

    m_Odometry->Reset(pose, m_Gyro->GetYawDegrees(), modulePositions);
    m_Pose = m_Odometry->GetWPIPose();
}

void SwerveDrive::Reset()
{
    ResetConstants();
    ResetEncoders();
}

void SwerveDrive::SampleSensors()
{
    std::array<CowLib::CowSwerveModulePosition, 4> modulePositions{};

    for (auto module : m_Modules)
    {
        modulePositions[module->GetID()] = module->GetPosition();
    }

    m_Odometry->Update(m_Gyro->GetYawDegrees(), modulePositions);

    m_Pose = m_Odometry->GetWPIPose();
}

void SwerveDrive::Handle()
{
    for (auto module : m_Modules)
    {
        module->Handle();
    }
}