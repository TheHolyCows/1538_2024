#include "ArmVisionCommand.h"

ArmVisionCommand::ArmVisionCommand(bool waitForCompletion)
{
    m_WaitForCompletion = waitForCompletion;
}

// ArmVisionCommand::ArmVisionCommand(double setpoint, ARM_SUBSYS subsystem, bool waitForCompletion)
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


bool ArmVisionCommand::IsComplete(CowRobot *robot)
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

void ArmVisionCommand::Start(CowRobot *robot)
{
    robot->m_Pivot->SetAngle(CONSTANT("PIVOT_AUTORANGING_SETPOINT"));

    frc::Pose2d lookaheadPose = robot->GetDrivetrain()->Odometry()->Lookahead(CONSTANT("POSE_LOOKAHEAD_TIME"))
                                                                    .value_or(robot->GetDrivetrain()->GetPose());
    
    double dist = robot->m_Vision->GetTargetDist(robot->m_Alliance, lookaheadPose);
    double wristSetpoint = (CONSTANT("WRIST_AUTO_RANGING_A") * std::pow(dist, 3)) +
                            (CONSTANT("WRIST_AUTO_RANGING_B") * std::pow(dist, 2)) +
                            (CONSTANT("WRIST_AUTO_RANGING_C") * std::pow(dist, 1)) +
                            (CONSTANT("WRIST_AUTO_RANGING_D") * std::pow(dist, 0)) +
                            CONSTANT("WRIST_STATIC_BIAS");

    robot->m_Shooter->PrimeShooter(robot->m_ShooterRangeMap[dist]);

    robot->m_Wrist->SetAngle(wristSetpoint + robot->m_BiasForAuto, robot->m_Pivot->GetSetpoint());
}

void ArmVisionCommand::Handle(CowRobot *robot)
{
    return;
}

void ArmVisionCommand::Finish(CowRobot *robot)
{
    return;
}