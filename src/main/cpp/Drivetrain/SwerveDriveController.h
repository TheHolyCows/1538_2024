#pragma once

#include "../CowLib/CowExponentialFilter.h"
#include "../CowPigeon.h"
#include "SwerveDrive.h"

#include <frc/controller/ProfiledPIDController.h>
#include <memory>
#include <optional>
#include <variant>

class SwerveDriveController
{
public:
    enum class RobotSide
    {
        FRONT,
        RIGHT,
        BACK,
        LEFT
    };

    struct DriveManualRequest
    {
        double inputX = 0;
        double inputY = 0;
        double inputRotation = 0;
    };

    struct DriveLockHeadingRequest
    {
        double inputX = 0;
        double inputY = 0;
    };

    struct DriveLookAtRequest
    {
        double inputX = 0;
        double inputY = 0;
        double targetX = 0;
        double targetY = 0;
        RobotSide robotSide = RobotSide::FRONT;
    };

    SwerveDriveController(SwerveDrive &drivetrain);
    ~SwerveDriveController() = default;

    void ResetConstants();
    void ResetHeadingLock();

    bool IsOnTarget();

    void Request(DriveManualRequest req);
    void Request(DriveLockHeadingRequest req);
    void Request(DriveLookAtRequest req);

    void Handle();

private:
    struct IdleState
    {
        
    };

    struct DriveManualState
    {
        DriveManualRequest req;
        std::optional<double> targetHeading;
    };

    struct DriveLockHeadingState
    {
        DriveLockHeadingRequest req;
        std::optional<double> targetHeading;
    };

    struct DriveLookAtState
    {
        DriveLookAtRequest req;
    };

    SwerveDrive &m_Drivetrain;
    CowPigeon &m_Gyro;

    std::variant<IdleState, DriveManualState, DriveLockHeadingState, DriveLookAtState> m_State;
    frc::ProfiledPIDController<units::degrees> m_HeadingPIDController;
    bool m_IsOnTarget;
};