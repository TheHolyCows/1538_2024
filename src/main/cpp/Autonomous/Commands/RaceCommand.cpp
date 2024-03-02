#include "RaceCommand.h"

RaceCommand::RaceCommand(std::vector<RobotCommand*> commands)
{
    m_Commands = commands;
}

RaceCommand::~RaceCommand()
{
    // TODO: figure out if these need to be deleted here (don't think so)

    // delete m_LeadCommand;
    // for (RobotCommand* command : m_OtherCommands)
    // {
    //     delete command;
    // }
}

bool RaceCommand::IsComplete(CowRobot *robot)
{
    for (RobotCommand* command : m_Commands) {
        if (command->IsComplete(robot)) {
            return true;
        }
    }
}

void RaceCommand::Start(CowRobot* robot)
{
    for (RobotCommand* command : m_Commands) {
        command->Start(robot);
    }
}

void RaceCommand::Handle(CowRobot* robot)
{
    for (RobotCommand* command : m_Commands) {
        command->Handle(robot);
    }
}

void RaceCommand::Finish(CowRobot* robot)
{
    for (RobotCommand* command : m_Commands) {
        command->Finish(robot);
    }
}