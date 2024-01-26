#pragma once

#include "../../CowLib/CowTimer.h"
#include "../../CowRobot.h"
#include "RobotCommand.h"

#include <frc/controller/HolonomicDriveController.h>
#include <frc/geometry/Pose2d.h>
#include <units/angle.h>
#include <units/time.h>

class PathToPoseCommand : public RobotCommand
{
private:
    CowLib::CowTimer *m_Timer;

    frc::PIDController *m_XController;
    frc::PIDController *m_YController;
    frc::PIDController *m_RotationController;
    

    double m_TotalTime;
    bool m_Stop;
    bool m_ResetOdometry;

    frc::Pose2d m_TargetPose;

public:
    PathToPoseCommand(units::second_t, frc::Pose2d targetPose, bool stopAtEnd = true, bool resetOdometry = false);
    ~PathToPoseCommand() override;

    bool IsComplete(CowRobot *robot) override;

    void Start(CowRobot *robot) override;

    void Handle(CowRobot *robot) override;

    void Finish(CowRobot *robot) override;
};
