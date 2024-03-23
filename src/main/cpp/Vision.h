#pragma once

#include "Cowconstants.h"
#include <frc/geometry/Pose2d.h>
#include <frc/Timer.h>
#include <frc/geometry/Pose3d.h>
#include <frc/DriverStation.h>
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <ctre/phoenix/led/CANdle.h>
#include <ctre/phoenix/led/RainbowAnimation.h>

class Vision {
public:
    struct Sample
    {
        units::second_t timestamp;
        frc::Pose3d pose3d;
        uint tagCount;

        auto operator<=>(const Sample&) const = default;
    };

    enum class LEDState
    {
        OFF,
        HOLD,
        ON_TARGET,
        INTAKING
    };

    Vision();

    ctre::phoenix::led::CANdle *m_CANdle;
    ctre::phoenix::led::RainbowAnimation *m_Rainbow;

    void ResetConstants();

    std::vector<Sample> GetRobotPose();
    void SetLEDState(LEDState ledState);
    void SampleSensors();
    void Handle();

    const frc::Translation2d BLUE_SPEAKER = { 0_ft, 18.2016666667_ft };
    const frc::Translation2d RED_SPEAKER = { 54.3941666667_ft, 18.2016666667_ft };

    double GetTargetDist(std::optional<frc::DriverStation::Alliance> alliance, frc::Pose2d lookaheadPose);
    frc::Translation2d GetTargetXY(std::optional<frc::DriverStation::Alliance> alliance);
private:
    std::vector<std::unique_ptr<photon::PhotonPoseEstimator>> m_PoseEstimators;
    std::vector<std::shared_ptr<photon::PhotonCamera>> m_Cameras;

    std::vector<photon::EstimatedRobotPose> m_EstimatedPoses;
    nt::IntegerTopic m_NTLEDState;
    LEDState m_LEDState;
    LEDState m_LEDBlinkInterval;
    int m_TickCount;
    double m_StateChangeTime;

    void SetLEDHold();
    void SetLEDOnTarget();
    void SetLEDOff();
};