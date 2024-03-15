#include "UpdateIntakeStateCommand.h"

UpdateIntakeStateCommand::UpdateIntakeStateCommand(Shooter::IntakeState state, bool waitForCompletion)
{
    m_State             = state;
    m_WaitForCompletion = waitForCompletion;
}

// due to possibility of missing the game piece, should probably run this as a race comamand with the
//   lead command being a wait command
bool UpdateIntakeStateCommand::IsComplete(CowRobot *robot)
{
    if (!m_WaitForCompletion)
    {
        return true;
    }

    if (m_State.has_value() && *m_State == Shooter::IntakeState::DETECT_ACTIVE)
    {
        if (robot->m_Shooter->GetIntakeState() == Shooter::IntakeState::DETECT_HOLD)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    return true;
}

void UpdateIntakeStateCommand::Start(CowRobot *robot)
{
    Shooter::IntakeState state = robot->m_Shooter->GetIntakeState();

    if (m_State.has_value())
    {
        state = *m_State;
    }

    robot->m_Shooter->UpdateIntakeState(state);
}

void UpdateIntakeStateCommand::Handle(CowRobot *robot)
{
    return;
}

void UpdateIntakeStateCommand::Finish(CowRobot *robot)
{
    return;
}