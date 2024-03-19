#pragma once

#include "../../CowRobot.h"
#include "../../Subsystems/Shooter.h"
#include "RobotCommand.h"

class UpdateShooterSpeed : public RobotCommand
{
private:
    double m_Dist;

public:
    // distance in feet to goal
    UpdateShooterSpeed(double dist);
    ~UpdateShooterSpeed() = default;

    bool IsComplete(CowRobot *robot) override;

    void Start(CowRobot *robot) override;

    void Handle(CowRobot *robot) override;

    void Finish(CowRobot *robot) override;
};