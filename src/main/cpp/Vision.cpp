#include "Vision.h"

#include <iostream>
#include <frc/apriltag/AprilTagFieldLayout.h>

Vision::Vision()
    : m_TickCount(0)
{
    std::vector<std::string> cameraNames = {"left", "center", "right"};

    for (const std::string& cameraName : cameraNames)
    {
        m_PoseEstimators.push_back(std::make_unique<photon::PhotonPoseEstimator>(
            frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo),
            photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
            photon::PhotonCamera{cameraName},
            frc::Transform3d()));
        m_PoseEstimators.back()->SetMultiTagFallbackStrategy(photon::PoseStrategy::LOWEST_AMBIGUITY);

        m_Cameras.push_back(m_PoseEstimators.back()->GetCamera());
        m_Cameras.back()->SetDriverMode(false);
        m_Cameras.back()->SetPipelineIndex(1);
    }

    ResetConsatnts();
}

void Vision::ResetConsatnts()
{
    // Left
    frc::Transform3d robotToLeftCamera(
        frc::Translation3d(-10.527929_in, 10.527929_in, 7.950596_in),
        frc::Rotation3d(0_deg, 20_deg, 135_deg));

    m_PoseEstimators.at(0)->SetRobotToCameraTransform(robotToLeftCamera);

    // Center
    frc::Transform3d robotToCenterCamera(
        frc::Translation3d(-11.064_in, 0.0_in, 6.950596_in),
        frc::Rotation3d(0_deg, 20_deg, 180_deg));

    m_PoseEstimators.at(1)->SetRobotToCameraTransform(robotToCenterCamera);

    // Right
    frc::Transform3d robotToRightCamera(
        frc::Translation3d(-10.527929_in, -10.527929_in, 7.950596_in),
        frc::Rotation3d(0_deg, 20_deg, 225_deg));

    m_PoseEstimators.at(2)->SetRobotToCameraTransform(robotToRightCamera);
}

std::vector<Vision::Sample> Vision::GetRobotPose()
{
    std::vector<Vision::Sample> samples;

    for (const photon::EstimatedRobotPose& estimatedPose : m_EstimatedPoses)
    {
        Vision::Sample sample;

        sample.timestamp = estimatedPose.timestamp;
        sample.pose3d = estimatedPose.estimatedPose;
        sample.tagCount = estimatedPose.targetsUsed.size();

        samples.push_back(sample);
    }

    return samples;
}

void Vision::SetLEDState(Vision::LEDState ledState)
{
    m_LEDState = ledState;
}

void Vision::SetLEDOn()
{
    // if (m_Camera->GetLEDMode() != photon::LEDMode::kOn)
    // {
    //     m_Camera->SetLEDMode(photon::LEDMode::kOn);
    // }
}

void Vision::SetLEDOff()
{
    // if (m_Camera->GetLEDMode() != photon::LEDMode::kOff)
    // {
    //     m_Camera->SetLEDMode(photon::LEDMode::kOff);
    // }
}

double Vision::GetTargetDist(std::optional<frc::DriverStation::Alliance> alliance, frc::Pose2d lookaheadPose)
{
    // Pivot and wrist targetting
    double robotX = lookaheadPose.X().convert<units::foot>().value();
    double robotY = lookaheadPose.Y().convert<units::foot>().value();

    frc::Translation2d targetXY = GetTargetXY(alliance);

    // this originally got the distance to the goal regardless of offsets, now it uses offsets
    // not sure if that is correct
    double dist = sqrtf(powf(units::foot_t(targetXY.Y()).value() - robotY, 2) + powf(units::foot_t(targetXY.X()).value() - robotX, 2));

    return dist;
}

frc::Translation2d Vision::GetTargetXY(std::optional<frc::DriverStation::Alliance> alliance)
{
    // maybe swap this to look at which half of the field we're on
    if (alliance.has_value())
    {
        if (alliance.value() == frc::DriverStation::Alliance::kRed)
        {
            return { units::foot_t(RED_SPEAKER.X()) - units::foot_t(CONSTANT("RED_GOAL_X_OFFSET")),
                     units::foot_t(RED_SPEAKER.Y()) - units::foot_t(CONSTANT("RED_GOAL_Y_OFFSET"))};

        }
        else
        {
            return { units::foot_t(BLUE_SPEAKER.X()) - units::foot_t(CONSTANT("BLUE_GOAL_X_OFFSET")),
                     units::foot_t(BLUE_SPEAKER.Y()) - units::foot_t(CONSTANT("BLUE_GOAL_Y_OFFSET"))};
        }

    }

    return { 0_ft, 0_ft };
}

void Vision::SampleSensors()
{
    m_EstimatedPoses.clear();

    for (const std::unique_ptr<photon::PhotonPoseEstimator>& poseEstimator : m_PoseEstimators)
    {
        std::optional<photon::EstimatedRobotPose> estimatedPose = poseEstimator->Update();

        if (estimatedPose.has_value())
        {
            m_EstimatedPoses.push_back(estimatedPose.value());
        }
    }
}

void Vision::Handle()
{
    if (m_LEDState == LEDState::OFF)
    {
        SetLEDOff();
    }
    else if (m_LEDState == LEDState::BLINK_SLOW)
    {
        if (m_TickCount == 1)
        {
            SetLEDOn();
        }
        else if (m_TickCount == CONSTANT("BLINK_SLOW_INTERVAL"))
        {
            SetLEDOff();
        }
        else if ( m_TickCount > CONSTANT("BLINK_SLOW_INTERVAL") * 2)
        {
            m_TickCount = 0;
        }
    }
    else if (m_LEDState == LEDState::BLINK_FAST)
    {
        if (m_TickCount == 1)
        {
            SetLEDOn();
        }
        else if (m_TickCount == CONSTANT("BLINK_FAST_INTERVAL"))
        {
            SetLEDOff();
        }
        else if (m_TickCount > CONSTANT("BLINK_FAST_INTERVAL") * 2)
        {
            m_TickCount = 0;
        }
    }

    m_TickCount++;
}