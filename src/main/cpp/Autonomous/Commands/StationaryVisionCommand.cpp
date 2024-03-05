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
        robot->GetDrivetrain()->SetVelocity(0, 0, 0, true);
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
    // TODO: Move to constants
    double goalX = 54.3941666667;
    double goalY = 18.2016666667;

    frc::Pose2d lookaheadPose = robot->GetDrivetrain()->Odometry()->Lookahead(
            CONSTANT("POSE_LOOKAHEAD_TIME")).value_or(robot->GetDrivetrain()->GetPose());

    double robotX = robot->GetDrivetrain()->GetPoseX();
    double robotY = robot->GetDrivetrain()->GetPoseY();

    SwerveDriveController::DriveLookAtRequest req = {
        .inputX = 0.0,
        .inputY = 0.0,
        .targetX = goalX - 1.0,
        .targetY = goalY,
        .robotSide = SwerveDriveController::RobotSide::BACK
    };

    robot->GetDriveController()->Request(req);

    double dist = sqrtf(powf(goalY - robotY, 2) + powf(goalX - robotX, 2));
    double rangePivot =robot->m_PivotRangeMap[dist];

    
    robot->m_Pivot->SetAngle(CONSTANT("PIVOT_AUTORANGING_SETPOINT"));
    robot->m_Wrist->SetAngle(rangePivot,robot->m_Pivot->GetSetpoint());
}

void StationaryVisionCommand::Finish(CowRobot *robot)
{
    m_Timer->Stop();
}
