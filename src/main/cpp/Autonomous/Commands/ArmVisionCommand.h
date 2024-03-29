#pragma once

#include "../../CowRobot.h"
#include "../../Subsystems/Wrist.h"
#include "../../Subsystems/Pivot.h"
#include "RobotCommand.h"

#include <optional>

class ArmVisionCommand : public RobotCommand
{
private:
    bool m_WaitForCompletion;

public:

    enum ARM_SUBSYS
    {
        SUB_WRIST = 0x0,
        SUB_PIVOT
    };

    ArmVisionCommand(bool waitForCompletion);
    // ArmVisionCommand(double setpoint, ARM_SUBSYS subsystem, bool waitForCompletion);
    ~ArmVisionCommand() = default;

    bool IsComplete(CowRobot *robot) override;

    void Start(CowRobot *robot) override;

    void Handle(CowRobot *robot) override;

    void Finish(CowRobot *robot) override;
};