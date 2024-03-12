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

    // Pivot and wrist targetting
    frc::Pose2d lookaheadPose = robot->GetDrivetrain()->Odometry()->Lookahead(CONSTANT("POSE_LOOKAHEAD_TIME")).value_or(robot->GetDrivetrain()->GetPose());

    double robotX = lookaheadPose.X().convert<units::foot>().value();
    double robotY = lookaheadPose.Y().convert<units::foot>().value();

    double dist = sqrtf(powf(CONSTANT("GOAL_Y") - robotY, 2) + powf(CONSTANT("GOAL_X") - robotX, 2));
    double wristBiasAngle = -3.0;
    double rangePivot = robot->m_PivotRangeMap[dist] + wristBiasAngle;

    robot->m_Pivot->SetAngle(CONSTANT("PIVOT_AUTORANGING_SETPOINT"));
    robot->m_Wrist->SetAngle(rangePivot + CONSTANT("WRIST_OFFSET_BIAS"), robot->m_Pivot->GetSetpoint());

    SwerveDriveController::DriveLookAtRequest req = {
        .inputX = 0.0,
        .inputY = 0.0,
        .targetX = goalX - 1.0,
        .targetY = goalY,
        .robotSide = SwerveDriveController::RobotSide::BACK
    };

    robot->GetDriveController()->Request(req);

}

void StationaryVisionCommand::Finish(CowRobot *robot)
{
    m_Timer->Stop();
}
