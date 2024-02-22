#pragma once

#include "Cowconstants.h"
#include <frc/geometry/Pose2d.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/Timer.h>

class Vision {
public:
    enum class LEDState
    {
        OFF,
        BLINK_SLOW,
        BLINK_FAST
    };
    
    struct PoseWithLatency 
    { 
        frc::Pose2d pose2d;
        double totalLatency;
    };
    
    Vision();
    PoseWithLatency GetRobotPose();
    void SetLEDState(LEDState ledState);
    void Handle();

private:
    LEDState m_LEDState;
    LEDState m_LEDBlinkInterval;
    double m_StateChangeTime;
    bool m_IsLEDOn;
};