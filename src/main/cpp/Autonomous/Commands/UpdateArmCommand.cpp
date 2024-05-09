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
    if (robot->m_Pivot->IsOnTarget())
    {
        return true;
    }

    return false;
}

void UpdateArmCommand::Start(CowRobot *robot)
{

    if (m_PivotSetpoint.has_value())
    {
        robot->m_Pivot->SetTargetAngle(m_PivotSetpoint.value());
        // printf("pivot: %f\n",m_PivotSetpoint.value());
    }

    if (m_WristSetpoint.has_value())
    {
        if (m_UseDistForWrist)
        {
            double dist = m_WristSetpoint.value();
            double wristSetpoint = (CONSTANT("WRIST_AUTO_RANGING_A") * std::pow(dist, 3)) +
                               (CONSTANT("WRIST_AUTO_RANGING_B") * std::pow(dist, 2)) +
                               (CONSTANT("WRIST_AUTO_RANGING_C") * std::pow(dist, 1)) +
                               (CONSTANT("WRIST_AUTO_RANGING_D") * std::pow(dist, 0)) +
                               CONSTANT("WRIST_STATIC_BIAS");

            robot->m_Wrist->SetAngle(wristSetpoint + robot->m_BiasForAuto, robot->m_Pivot->GetTargetAngle());
        }
        else
        {
            robot->m_Wrist->SetAngle(m_WristSetpoint.value() + robot->m_BiasForAuto, robot->m_Pivot->GetTargetAngle());
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