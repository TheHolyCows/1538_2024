#include "UpdateArmCommand.h"

UpdateArmCommand::UpdateArmCommand(double wristSetpoint, double pivotSetpoint, bool waitForCompletion, bool useDistForWrist)
{
    m_WristSetpoint = wristSetpoint;
    m_PivotSetpoint = pivotSetpoint;
    m_WaitForCompletion = waitForCompletion;
    m_UseDistForWrist = useDistForWrist;
}

// UpdateArmCommand::UpdateArmCommand(double setpoint, ARM_SUBSYS subsystem, bool waitForCompletion)
// {
//     if (subsystem == ARM_SUBSYS::SUB_WRIST)
//     {
//         m_WristSetpoint = setpoint;
//     }
//     else if (subsystem == ARM_SUBSYS::SUB_PIVOT)
//     {
//         m_PivotSetpoint = setpoint;
//     }
    
//     m_WaitForCompletion = waitForCompletion;
// }


bool UpdateArmCommand::IsComplete(CowRobot *robot)
{
    if (!m_WaitForCompletion)
    {
        return true;
    }

    // gonna ignore wrist since that is less important
    if (robot->m_Pivot->AtTarget())
    {
        return true;
    }

    return false;
}

void UpdateArmCommand::Start(CowRobot *robot)
{

    if (m_PivotSetpoint.has_value())
    {
        robot->m_Pivot->SetAngle(m_PivotSetpoint.value());
        // printf("pivot: %f\n",m_PivotSetpoint.value());
    }

    if (m_WristSetpoint.has_value())
    {
        if (m_UseDistForWrist)
        {
            robot->m_Wrist->SetAngle(robot->m_PivotRangeMap[m_WristSetpoint.value()] + robot->m_BiasForAuto, robot->m_Pivot->GetSetpoint());
        }
        else
        {
            robot->m_Wrist->SetAngle(m_WristSetpoint.value() + robot->m_BiasForAuto, robot->m_Pivot->GetSetpoint());
        }
    }
}

void UpdateArmCommand::Handle(CowRobot *robot)
{
    return;
}

void UpdateArmCommand::Finish(CowRobot *robot)
{
    return;
}