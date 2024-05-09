#pragma once

#include "../../CowLib/CowTimer.h"
#include "../../CowRobot.h"
#include "./RobotCommand.h"
#include "CowLibTrajectory.h"
#include "../../CowLib/Pathplanner/PathplannerUtils.h"

#include <iostream>
#include <frc/Filesystem.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/path/PathPlannerTrajectory.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <string>
#include <vector>
#include <wpi/json.h>
#include <fstream>

class PathplannerSwerveCommand : public RobotCommand
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

    PathplannerSwerveCommand(const std::string &pathName,
                                       units::feet_per_second_t maxVelocity,
                                       units::feet_per_second_squared_t maxAccel,
                                       bool stop,
                                       bool overrideInitPose        = false);
    ~PathplannerSwerveCommand() override;

    bool IsComplete(CowRobot *robot) override;

    void Start(CowRobot *robot) override;

    void Handle(CowRobot *robot) override;

    void Finish(CowRobot *robot) override;

private:
    CowLib::CowTimer *m_Timer;

    wpi::json m_PathData;

    std::shared_ptr<pathplanner::PathPlannerPath> m_Path;
    std::shared_ptr<pathplanner::CowLibTrajectory> m_Trajectory;
    pathplanner::PPHolonomicDriveController *m_HolonomicController;

    double m_TotalTime;
    bool m_Stop;
    bool m_OverrideInitPose;
};
