#ifndef __SWERVE_DRIVE_H__
#define __SWERVE_DRIVE_H__

#include "../CowConstants.h"
#include "../CowLib/Swerve/CowSwerveKinematics.h"
#include "../CowLib/Swerve/CowSwerveModulePosition.h"
#include "../CowLib/Swerve/CowSwerveOdometry.h"
#include "../CowLib/Utility.h"
#include "../CowPigeon.h"
#include "SwerveModule.h"
#include "SwerveModuleSim.h"
#include "SwerveModuleInterface.h"
#include "../Vision.h"

#include <algorithm>
#include <array>
#include <iostream>
#include <memory>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/RobotBase.h>

class SwerveDrive
{
private:
    std::array<SwerveModuleInterface *, 4> m_Modules{};
    CowPigeon *m_Gyro;

    CowLib::CowSwerveKinematics *m_Kinematics;
    std::shared_ptr<CowLib::CowSwerveOdometry> m_Odometry;
    frc::Pose2d m_Pose;
    frc::ChassisSpeeds m_PrevChassisSpeeds;

    bool m_Locked;

public:
    struct ModuleConstants
    {
        int driveMotorId;
        int rotationMotorId;
        int encoderId;
        double encoderOffset;
    };

    SwerveDrive(ModuleConstants constants[4], double wheelBase);
    ~SwerveDrive();

    std::vector<ctre::phoenix6::BaseStatusSignal*> GetSynchronizedSignals();
    std::shared_ptr<CowLib::CowSwerveOdometry> Odometry();

    void SetVelocity(double x,
                     double y,
                     double rotation,
                     bool isFieldRelative     = true,
                     double centerOfRotationX = 0,
                     double centerOfRotationY = 0,
                     bool force               = false);

    void SetVelocity(CowLib::CowChassisSpeeds chassisSpeeds,
                     bool isFieldRelative     = true,
                     double centerOfRotationX = 0,
                     double centerOfRotationY = 0,
                     bool force               = false);
    
    frc::Pose2d GetPose();
    frc::ChassisSpeeds GetChassisSpeeds();

    double GetPoseX();
    double GetPoseY();
    double GetPoseRot();

    bool GetLocked() const;
    void SetLocked(bool isLocked);

    void SetBrakeMode(bool brakeMode);

    void AddVisionMeasurement(Vision::Sample sample);

    void ResetConstants();
    void ResetEncoders();
    void ResetOdometry(frc::Pose2d pose = frc::Pose2d{ 0_m, 0_m, 0_deg });
    void Reset();

    void Handle();
};

#endif /* __SWERVE_DRIVE_H__ */