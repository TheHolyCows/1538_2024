#pragma once

#include "../../CowLib/CowTimer.h"
#include "../../CowRobot.h"
#include "RobotCommand.h"

#include <frc/controller/HolonomicDriveController.h>
#include <frc/geometry/Pose2d.h>
#include <units/angle.h>
#include <units/time.h>

class RotateCommand : public RobotCommand
{
private:
    CowLib::CowTimer *m_Timer;

    frc::PIDController *m_RotationController;

    double m_TotalTime;
    bool m_Stop;
    bool m_ResetOdometry;

    units::degree_t m_TargetRotation;

    frc::Pose2d m_Pose;
    frc::Pose2d m_TargetPose;

public:
    RotateCommand(units::second_t, units::degree_t targetRotation, bool stopAtEnd = true, bool resetOdometry = false);
    ~RotateCommand() override;

    bool IsComplete(CowRobot *robot) override;

    void Start(CowRobot *robot) override;

    void Handle(CowRobot *robot) override;

    void Finish(CowRobot *robot) override;
};
