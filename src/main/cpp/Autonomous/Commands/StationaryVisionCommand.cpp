#include "StationaryVisionCommand.h"

StationaryVisionCommand::StationaryVisionCommand(units::second_t timeout)
    : m_Timer(std::make_unique<CowLib::CowTimer>()),
      m_Gyro(*CowPigeon::GetInstance()),
      m_Timeout(timeout)
{
}

bool StationaryVisionCommand::IsComplete(CowRobot *robot)
{
    if (m_Timer->HasElapsed(m_Timeout.value()))
    {
        SwerveDriveController::DriveManualRequest req = {
            .inputX = 0,
            .inputY = 0,
            .inputRotation = 0
        };

        robot->GetDriveController()->Request(req);
        return true;
    }

    return false;
}

void StationaryVisionCommand::Start(CowRobot *robot)
{
    m_Timer->Start();
}

void StationaryVisionCommand::Handle(CowRobot *robot)
{
    // Pivot and wrist targetting
    frc::Pose2d lookaheadPose = robot->GetDrivetrain()->Odometry()->Lookahead(CONSTANT("POSE_LOOKAHEAD_TIME"))
                                                                    .value_or(robot->GetDrivetrain()->GetPose());
    double dist = robot->m_Vision->GetTargetDist(robot->m_Alliance, lookaheadPose);

    double wristBias = robot->m_BiasForAuto;

    double wristSetpoint = (CONSTANT("WRIST_AUTO_RANGING_A") * std::pow(dist, 3)) +
                               (CONSTANT("WRIST_AUTO_RANGING_B") * std::pow(dist, 2)) +
                               (CONSTANT("WRIST_AUTO_RANGING_C") * std::pow(dist, 1)) +
                               (CONSTANT("WRIST_AUTO_RANGING_D") * std::pow(dist, 0)) +
                               CONSTANT("WRIST_STATIC_BIAS");

    robot->m_Wrist->SetAngle(wristSetpoint + robot->m_BiasForAuto, robot->m_Pivot->GetAngle());

    robot->m_Pivot->SetTargetAngle(CONSTANT("PIVOT_AUTORANGING_SETPOINT_HI"));
    // robot->m_Wrist->SetAngle(rangePivot + CONSTANT("WRIST_OFFSET_BIAS"), robot->m_Pivot->GetSetpoint());

    robot->m_Shooter->PrimeShooter(robot->m_ShooterRangeMap[dist]);

    frc::Translation2d targetXY = robot->m_Vision->GetTargetXY(robot->m_Alliance);

        SwerveDriveController::DriveLookAtRequest req = {
            .inputX = 0.0,
            .inputY = 0.0,
            .targetX = units::foot_t(targetXY.X()).value(),
            .targetY = units::foot_t(targetXY.Y()).value(),
            .robotSide = SwerveDriveController::RobotSide::BACK,
            .lookaheadTime = CONSTANT("POSE_LOOKAHEAD_TIME")
        };

    robot->GetDriveController()->Request(req);

}

void StationaryVisionCommand::Finish(CowRobot *robot)
{
    m_Timer->Stop();
}
