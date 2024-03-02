#pragma once

#include "Cowconstants.h"
#include <frc/geometry/Pose2d.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/Timer.h>
#include <frc/geometry/Pose3d.h>

class Vision {
public:
    enum class LEDState
    {
        OFF,
        BLINK_SLOW,
        BLINK_FAST
    };
    
    struct Sample 
    { 
        frc::Pose3d pose3d;
        units::second_t totalLatency;
        uint tagCount;
        double tagSpan;
        double averageTagDistance;
        double averageTagArea;
    };
    
    Vision();
    Sample GetRobotPose();
    void SetLEDState(LEDState ledState);
    void LEDOn();
    void LEDOff();
    void Handle();

private:
    LEDState m_LEDState;
    LEDState m_LEDBlinkInterval;
    double m_StateChangeTime;
    bool m_IsLEDOn;
    int m_TickCount;
};