#pragma once

#include "../../CowRobot.h"
#include "../../Subsystems/Shooter.h"
#include "RobotCommand.h"

#include <optional>

class UpdateIntakeStateCommand : public RobotCommand
{
private:
    std::optional<Shooter::IntakeState> m_State = std::nullopt;
    bool m_WaitForCompletion;

public:
    UpdateIntakeStateCommand(Shooter::IntakeState state, bool waitForCompletion);
    ~UpdateIntakeStateCommand() = default;

    bool IsComplete(CowRobot *robot) override;

    void Start(CowRobot *robot) override;

    void Handle(CowRobot *robot) override;

    void Finish(CowRobot *robot) override;
};