#include "UpdateShooterSpeed.h"

UpdateShooterSpeed::UpdateShooterSpeed(double dist)
{
    m_Dist = dist;
}

bool UpdateShooterSpeed::IsComplete(CowRobot *robot)
{
    return true;
}

void UpdateShooterSpeed::Start(CowRobot *robot)
{
    robot->m_Shooter->PrimeShooter(robot->m_ShooterRangeMap[m_Dist]);
}

void UpdateShooterSpeed::Handle(CowRobot *robot)
{
    return;
}

void UpdateShooterSpeed::Finish(CowRobot *robot)
{
    return;
}