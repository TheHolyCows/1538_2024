#pragma once

#include <frc/geometry/Pose2d.h>
#include <networktables/NetworkTableInstance.h>

class Vision {
public:
    Vision();
    frc::Pose2d GetRobotPose();
    void SetLEDState();
    enum class LEDState
    {
        OFF,
        BLINK_SLOW,
        BLINK_FAST
    };
private:
};