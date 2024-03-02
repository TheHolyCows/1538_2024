#ifndef __WAIT_COMMAND_H__
#define __WAIT_COMMAND_H__

#include "../../CowLib/CowTimer.h"
#include "../../CowRobot.h"
#include "RobotCommand.h"

class WaitCommand : public RobotCommand {
private:
    CowLib::CowTimer* m_Timer;

    bool m_DoNothing;

    units::second_t m_TimeToWait;

public:
    explicit WaitCommand(units::second_t timeToWait, bool doNothing);
    ~WaitCommand() override;

    bool IsComplete(CowRobot *robot) override;

    void Start(CowRobot* robot) override;

    void Handle(CowRobot* robot) override;

    void Finish(CowRobot* robot) override;
};

#endif /* __WAIT_COMMAND_H__ */
