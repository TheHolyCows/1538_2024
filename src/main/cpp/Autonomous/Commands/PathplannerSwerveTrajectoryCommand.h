#pragma once

#include "../../CowLib/CowTimer.h"
#include "../../CowRobot.h"
#include "./RobotCommand.h"

#include <iostream>
#include <frc/Filesystem.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/path/PathPlannerTrajectory.h>
#include "CowLibTrajectory.h"
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <string>
#include <vector>
#include <wpi/json.h>
#include <fstream>

class PathplannerSwerveTrajectoryCommand : public RobotCommand
{
public:
    struct Event
    {
        std::string waypointName;
        RobotCommand *command;
        double time  = -1;
        bool done    = false;
        bool started = false;
    };

    PathplannerSwerveTrajectoryCommand(const std::string &trajectoryName,
                                       units::feet_per_second_t maxVelocity,
                                       units::feet_per_second_squared_t maxAccel,
                                       frc::Rotation2d startingRotation,
                                       bool stop,
                                       bool resetOdometry        = false,
                                       std::vector<Event> events = {});
    ~PathplannerSwerveTrajectoryCommand() override;

    bool IsComplete(CowRobot *robot) override;

    void Start(CowRobot *robot) override;

    void Handle(CowRobot *robot) override;

    void Finish(CowRobot *robot) override;

    frc::Pose2d GetStartingPose();
    
    frc::Rotation2d GetEndRot();

private:
    CowLib::CowTimer *m_Timer;

    std::shared_ptr<pathplanner::PathPlannerPath> m_Path;
    std::shared_ptr<pathplanner::CowLibTrajectory> m_Trajectory;
    pathplanner::PPHolonomicDriveController *m_HolonomicController;

    double m_TotalTime;
    bool m_Stop;
    bool m_ResetOdometry;

    frc::Pose2d m_StartPose;
    frc::Pose2d m_EndPose;
    frc::Rotation2d m_StartRotation;
    frc::Rotation2d m_EndRotation;
    
    std::vector<Event> m_Events;
};
