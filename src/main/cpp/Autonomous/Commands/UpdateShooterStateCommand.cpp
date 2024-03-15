#include "UpdateShooterStateCommand.h"

UpdateShooterStateCommand::UpdateShooterStateCommand(Shooter::ShooterState state, bool waitForCompletion)
{
    m_State             = state;
    m_WaitForCompletion = waitForCompletion;
}

bool UpdateShooterStateCommand::IsComplete(CowRobot *robot)
{
    if (!m_WaitForCompletion)
    {
        return true;
    }

    if (m_State.has_value() && *m_State == Shooter::ShooterState::SPIN_UP)
    {
        if (robot->m_Shooter->GetShooterState() == Shooter::ShooterState::READY)
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

void UpdateShooterStateCommand::Start(CowRobot *robot)
{
    Shooter::ShooterState state = robot->m_Shooter->GetShooterState();

    if (m_State.has_value())
    {
        state = *m_State;

        if (state == Shooter::ShooterState::SPIN_UP)
        {
            robot->m_Shooter->PrimeShooter(47);
        }
    }

    robot->m_Shooter->UpdateShooterState(state);
}

void UpdateShooterStateCommand::Handle(CowRobot *robot)
{
    return;
}

void UpdateShooterStateCommand::Finish(CowRobot *robot)
{
    return;
}