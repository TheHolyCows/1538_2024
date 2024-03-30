#include "PathplannerVisionCommand.h"

PathplannerVisionCommand::PathplannerVisionCommand(const std::string &pathName,
                                                    units::feet_per_second_t maxVelocity,
                                                    units::feet_per_second_squared_t maxAccel,
                                                    double startOverridePct,
                                                    double endOverridePct,
                                                    bool stopAtEnd,
                                                    bool overrideInitPose)
{
     // This is to make sure that it is loading trajectories on start and not on demand
    m_Timer         = new CowLib::CowTimer();
    m_Stop          = stopAtEnd;
    m_StartOverridePercent = startOverridePct;
    m_EndOverridePercent = endOverridePct;
    m_OverrideInitPose = overrideInitPose;


    // Load path from file
    m_Path = pathplanner::PathPlannerPath::fromPathFile(pathName);

    // read in path file and modify velocity and acceleration based on values passed to this function
    m_PathData = CowLib::ParsePathFile(pathName);

    CowLib::UpdatePathplannerVelocity(&m_PathData,maxVelocity);
    CowLib::UpdatePathplannerAcceleration(&m_PathData,maxAccel);

    m_Path->hotReload(m_PathData);

    // get poses and rotations for start and end
    // auto start_pose = m_PathData["waypoints"][0]["anchor"];
    // auto end_pose = m_PathData["waypoints"][m_PathData["waypoints"].size() - 1]["anchor"];


    m_HolonomicController = new pathplanner::PPHolonomicDriveController(
        pathplanner::PIDConstants{ CONSTANT("AUTO_DRIVE_P"), CONSTANT("AUTO_DRIVE_I"), CONSTANT("AUTO_DRIVE_D") },
        pathplanner::PIDConstants{ CONSTANT("AUTO_ROTATION_P"), CONSTANT("AUTO_ROTATION_I"), CONSTANT("AUTO_ROTATION_D") },
        units::meters_per_second_t(units::feet_per_second_t(CONSTANT("SWERVE_MAX_SPEED"))),
        units::meter_t(units::length::foot_t(CONSTANT("AUTO_DRIVE_BASE_RADIUS"))),
        0.01_s);
    m_HolonomicController->setEnabled(true);
}

PathplannerVisionCommand::~PathplannerVisionCommand()
{
    delete m_Timer;
    delete m_HolonomicController;
}

bool PathplannerVisionCommand::IsComplete(CowRobot *robot)
{
    return m_Timer->HasElapsed(m_TotalTime);
}

void PathplannerVisionCommand::Start(CowRobot *robot)
{
   frc::ChassisSpeeds curSpeeds = robot->GetDrivetrain()->GetChassisSpeeds();
    frc::Pose2d curPose = robot->GetDrivetrain()->GetPose();

    if (m_OverrideInitPose)
    {
        // this is where we take the pose from the robot (preferably vision)
        //   and override the starting point in the path with it
        //   if it is setting to 0,0 make sure we are calling SwerveDrive::Handle() in disabled
        m_PathData["waypoints"][0]["anchor"]["x"] = curPose.X().value();
        m_PathData["waypoints"][0]["anchor"]["y"] = curPose.Y().value();
        m_Path->hotReload(m_PathData);
    }

    m_Trajectory = std::make_shared<pathplanner::CowLibTrajectory>(m_Path, curSpeeds, curPose.Rotation());

    m_TotalTime = m_Trajectory->getTotalTime().value();

    m_HolonomicController->reset(curPose,curSpeeds);

    m_Timer->Reset();
    m_Timer->Start();
}

void PathplannerVisionCommand::Handle(CowRobot *robot)
{

    frc::Pose2d currentPose = robot->GetDrivetrain()->GetPose();

    pathplanner::PathPlannerTrajectory::State targetState
        = m_Trajectory->sample(units::second_t{ m_Timer->Get() });

    // this is called in the PathPlanner implementation of this method, not sure why
    // targetState = targetState.reverse();

    // override rotation
    double percentCompletion = m_Timer->Get() / m_TotalTime * 100;
    if (percentCompletion >= m_StartOverridePercent && percentCompletion <= m_EndOverridePercent)
    {
        frc::Translation2d targetXY = robot->m_Vision->GetTargetXY(robot->m_Alliance);
        double angleToTarget = std::atan2(targetXY.Y().convert<units::foot>().value() - robot->m_Drivetrain->GetPoseY(),
                                          targetXY.X().convert<units::foot>().value() - robot->m_Drivetrain->GetPoseX());
        targetState.targetHolonomicRotation = frc::Rotation2d { units::degree_t(angleToTarget) };

        // optionally include wrist?
    }


    CowLib::CowChassisSpeeds chassisSpeeds
        = CowLib::CowChassisSpeeds::FromWPI(m_HolonomicController->calculateRobotRelativeSpeeds(currentPose, targetState));


    robot->GetDrivetrain()->SetVelocity(chassisSpeeds, false);
}

void PathplannerVisionCommand::Finish(CowRobot *robot)
{
    if (m_Stop)
    {
        // CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "Stopping swerve trajectory command");
        robot->GetDrivetrain()->SetVelocity(0, 0, 0, true);
    }

    m_Timer->Stop();
}