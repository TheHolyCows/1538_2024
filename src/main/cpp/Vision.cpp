#include "Vision.h"

#include <iostream>
#include <frc/apriltag/AprilTagFieldLayout.h>

Vision::Vision()
//    : m_TickCount(0)
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

    m_CANdle = new ctre::phoenix::led::CANdle(31,"cowdrive");

    m_CANdle->SetLEDs(255, 255, 255);
    m_CANdle->ConfigBrightnessScalar(0);

    // m_Rainbow = new ctre::phoenix::led::RainbowAnimation(0.75, 0.75);


    ResetConstants();
}

void Vision::ResetConstants()
{
    // Left
    frc::Transform3d robotToLeftCamera(
        frc::Translation3d(-10.527929_in, 10.527929_in, 7.950596_in),
        frc::Rotation3d(0_deg, -20_deg, 135_deg));

    m_PoseEstimators.at(0)->SetRobotToCameraTransform(robotToLeftCamera);

    // Center
    frc::Transform3d robotToCenterCamera(
        frc::Translation3d(-11.064_in, 0.0_in, 6.950596_in),
        frc::Rotation3d(0_deg, -25_deg, 180_deg));

    m_PoseEstimators.at(1)->SetRobotToCameraTransform(robotToCenterCamera);

    // Right
    frc::Transform3d robotToRightCamera(
        frc::Translation3d(-10.527929_in, -10.527929_in, 7.950596_in),
        frc::Rotation3d(0_deg, -20_deg, 225_deg));

    m_PoseEstimators.at(2)->SetRobotToCameraTransform(robotToRightCamera);
}

std::vector<Vision::Sample> Vision::GetRobotPose()
{
    std::vector<Vision::Sample> samples;

    for (const photon::EstimatedRobotPose& estimatedPose : m_EstimatedPoses)
    {
        bool valid = true;

        // if (estimatedPose.targetsUsed.size() == 1 &&
        //     !estimatedPose.targetsUsed[0].GetFiducialId() == 5 &&
        //     !estimatedPose.targetsUsed[0].GetFiducialId() == 6 &&
        //     !estimatedPose.targetsUsed[0].GetFiducialId() == 13 &&
        //     !estimatedPose.targetsUsed[0].GetFiducialId() == 14)
        // {
        //     valid = false;
        // }

        if (valid) {
            Vision::Sample sample;

            sample.timestamp = estimatedPose.timestamp;
            sample.pose3d = estimatedPose.estimatedPose;
            sample.tagCount = estimatedPose.targetsUsed.size();

            samples.push_back(sample);
        }
    }

    return samples;
}

void Vision::SetLEDState(Vision::LEDState ledState)
{
    m_LEDState = ledState;
}

void Vision::SetLEDHold()
{
    if (m_LEDState == LEDState::HOLD)
    {
        m_CANdle->SetLEDs(CONSTANT("LED_HOLD_R"), CONSTANT("LED_HOLD_G"), CONSTANT("LED_HOLD_B"));
        m_CANdle->ConfigBrightnessScalar(CONSTANT("LED_BRIGHTNESS"));
    }
    // if (m_Camera->GetLEDMode() != photon::LEDMode::kOn)
    // {
    //     m_Camera->SetLEDMode(photon::LEDMode::kOn);
    // }
}

void Vision::SetLEDOnTarget()
{
    if (m_LEDState == LEDState::ON_TARGET)
    {
        m_CANdle->SetLEDs(CONSTANT("LED_ON_TARGET_R"), CONSTANT("LED_ON_TARGET_G"), CONSTANT("LED_ON_TARGET_B"));
        m_CANdle->ConfigBrightnessScalar(CONSTANT("LED_BRIGHTNESS"));
    }
}

void Vision::SetLEDOff()
{
    if (m_LEDState == LEDState::OFF)
    {
        m_CANdle->SetLEDs(255, 255, 255);
        m_CANdle->ConfigBrightnessScalar(0);
    }

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

double Vision::GetPassTargetDist(std::optional<frc::DriverStation::Alliance> alliance, frc::Pose2d lookaheadPose)
{
    return GetTargetDist(alliance, lookaheadPose) / 2.0;
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

frc::Translation2d Vision::GetPassTargetXY(std::optional<frc::DriverStation::Alliance> alliance)
{
    // maybe swap this to look at which half of the field we're on
    if (alliance.has_value())
    {
        if (alliance.value() == frc::DriverStation::Alliance::kRed)
        {
            return { units::foot_t(RED_CORNER.X()),
                     units::foot_t(RED_CORNER.Y())};

        }
        else
        {
            return { units::foot_t(BLUE_CORNER.X()),
                     units::foot_t(BLUE_CORNER.Y())};
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
            // printf("%s x: %f  y: %f\n",std::string(poseEstimator->GetCamera()->GetCameraName()).c_str(),
            //                                     estimatedPose.value().estimatedPose.X().value(),
            //                                     estimatedPose.value().estimatedPose.Y().value());
        }
    }
}

void Vision::Handle()
{
    if (m_LEDState == LEDState::OFF)
    {
        SetLEDOff();
    }
    else if (m_LEDState == LEDState::HOLD)
    {
        SetLEDHold();

        // if (m_TickCount == 1)
        // {
        //     SetLEDOn();
        // }
        // else if (m_TickCount == CONSTANT("BLINK_SLOW_INTERVAL"))
        // {
        //     SetLEDOff();
        // }
        // else if ( m_TickCount > CONSTANT("BLINK_SLOW_INTERVAL") * 2)
        // {
        //     m_TickCount = 0;
        // }
    }
    else if (m_LEDState == LEDState::ON_TARGET)
    {
        SetLEDOnTarget();

        // if (m_TickCount == 1)
        // {
        //     SetLEDOn();
        // }
        // else if (m_TickCount == CONSTANT("BLINK_FAST_INTERVAL"))
        // {
        //     SetLEDOff();
        // }
        // else if (m_TickCount > CONSTANT("BLINK_FAST_INTERVAL") * 2)
        // {
        //     m_TickCount = 0;
        // }
    }
    else if (m_LEDState == LEDState::INTAKING)
    {
        m_CANdle->SetLEDs(CONSTANT("LED_INTAKING_R"), CONSTANT("LED_INTAKING_G"), CONSTANT("LED_INTAKING_B"));
        m_CANdle->ConfigBrightnessScalar(CONSTANT("LED_BRIGHTNESS"));
    }
    else if (m_LEDState == LEDState::EXHAUSTING)
    {
        m_CANdle->SetLEDs(255, 0, 0);
        m_CANdle->ConfigBrightnessScalar(CONSTANT("LED_BRIGHTNESS"));
    }

    // m_TickCount++;
}