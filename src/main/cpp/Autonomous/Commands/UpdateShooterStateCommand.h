#pragma once

#include "../../CowRobot.h"
#include "../../Subsystems/Shooter.h"
#include "RobotCommand.h"

#include <optional>

class UpdateShooterStateCommand : public RobotCommand
{
private:
    std::optional<Shooter::ShooterState> m_State = std::nullopt;
    bool m_WaitForCompletion;

public:
    UpdateShooterStateCommand(Shooter::ShooterState state, bool waitForCompletion);
    ~UpdateShooterStateCommand() = default;

    bool IsComplete(CowRobot *robot) override;

    void Start(CowRobot *robot) override;

    void Handle(CowRobot *robot) override;

    void Finish(CowRobot *robot) override;
};