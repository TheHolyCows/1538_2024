#pragma once

#include "../CowLib/CowExponentialFilter.h"
#include "../CowPigeon.h"
#include "SwerveDrive.h"

#include <frc/controller/ProfiledPIDController.h>
#include <memory>

class SwerveDriveController
{
public:
    SwerveDriveController(SwerveDrive &drivetrain);
    ~SwerveDriveController() = default;

    void Drive(double x, double y, double rotation, bool fieldRelative);

    void DriveLookAt(double x, double y, double targetX, double targetY);

    // 2023
    // void CubeAlign(double x);
    // void ConeAlign(double x, double yInput);

    void LockHeading(double x, double y, bool useRawInputs=false);

    void ResetHeadingLock();

    void ResetConstants();

private:
    double ProcessDriveAxis(double input, double scaleMin, double scaleMax, bool reverse);

    SwerveDrive &m_Drivetrain;

    CowPigeon &m_Gyro;

    std::unique_ptr<CowLib::CowExponentialFilter> m_ExponentialFilter;

    std::unique_ptr<frc::ProfiledPIDController<units::degrees>> m_HeadingPIDController;

    bool m_HeadingLocked;
    double m_TargetHeading;
};