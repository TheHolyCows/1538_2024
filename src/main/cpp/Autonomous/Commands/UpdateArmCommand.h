#pragma once

#include "../../CowRobot.h"
#include "../../Subsystems/Wrist.h"
#include "../../Subsystems/Pivot.h"
#include "RobotCommand.h"

#include <optional>

class UpdateArmCommand : public RobotCommand
{
private:
    std::optional<double> m_WristSetpoint = std::nullopt;
    std::optional<double> m_PivotSetpoint = std::nullopt;
    bool m_WaitForCompletion;

public:

    enum ARM_SUBSYS
    {
        SUB_WRIST = 0x0,
        SUB_PIVOT
    };

    UpdateArmCommand(double wristSetpoint, double pivotSetpoint, bool waitForCompletion);
    UpdateArmCommand(double setpoint, ARM_SUBSYS subsystem, bool waitForCompletion);
    ~UpdateArmCommand() = default;

    bool IsComplete(CowRobot *robot) override;

    void Start(CowRobot *robot) override;

    void Handle(CowRobot *robot) override;

    void Finish(CowRobot *robot) override;
};