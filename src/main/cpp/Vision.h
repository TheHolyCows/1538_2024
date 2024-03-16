#pragma once

#include "Cowconstants.h"
#include <frc/geometry/Pose2d.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/Timer.h>
#include <frc/geometry/Pose3d.h>
#include <frc/DriverStation.h>

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

        auto operator<=>(const Sample&) const = default;
    };
    
    Vision();
    Sample GetRobotPose();
    void SetLEDState(LEDState ledState);
    void LEDOn();
    void LEDOff();
    void Handle();

    const frc::Translation2d BLUE_SPEAKER = { 0_ft, 18.2016666667_ft };
    const frc::Translation2d RED_SPEAKER = { 54.3941666667_ft, 18.2016666667_ft };


    double GetTargetDist(std::optional<frc::DriverStation::Alliance> alliance, frc::Pose2d lookaheadPose);
    frc::Translation2d GetTargetXY(std::optional<frc::DriverStation::Alliance> alliance);

private:
    LEDState m_LEDState;
    LEDState m_LEDBlinkInterval;
    int m_TickCount;
    double m_StateChangeTime;
    bool m_IsLEDOn;
};