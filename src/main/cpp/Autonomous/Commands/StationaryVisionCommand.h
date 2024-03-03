#pragma once

#include "../../CowLib/CowTimer.h"
#include "../../CowRobot.h"
#include "RobotCommand.h"
#include "../../CowPigeon.h"

#include <memory>

class StationaryVisionCommand : public RobotCommand
{
private:
    std::unique_ptr<CowLib::CowTimer> m_Timer;

    CowPigeon& m_Gyro;

    const units::second_t m_Timeout;

public:
    StationaryVisionCommand(units::second_t timeout);
    ~StationaryVisionCommand() override = default;

    bool IsComplete(CowRobot *robot) override;

    void Start(CowRobot *robot) override;

    void Handle(CowRobot *robot) override;

    void Finish(CowRobot *robot) override;
};