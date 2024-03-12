#include "PathplannerSwerveCommand.h"

PathplannerSwerveCommand::PathplannerSwerveCommand(const std::string &pathName,
                                                                       units::feet_per_second_t maxVelocity,
                                                                       units::feet_per_second_squared_t maxAccel,
                                                                       bool stopAtEnd,
                                                                       bool overrideInitPose)
{
    // This is to make sure that it is loading trajectories on start and not on demand
    m_Timer         = new CowLib::CowTimer();
    m_Stop          = stopAtEnd;
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

PathplannerSwerveCommand::~PathplannerSwerveCommand()
{
    delete m_Timer;
    delete m_HolonomicController;
}

bool PathplannerSwerveCommand::IsComplete(CowRobot *robot)
{
    return m_Timer->HasElapsed(m_TotalTime);
}

void PathplannerSwerveCommand::Start(CowRobot *robot)
{
    frc::ChassisSpeeds curSpeeds = robot->GetDrivetrain()->GetChassisSpeeds();
    frc::Pose2d curPose = robot->GetDrivetrain()->GetPose();

    if (m_OverrideInitPose)
    {
        // this is where we take the pose from the robot (preferably vision)
        //   and override the starting point in the path with it
        //   if it is setting to 0,0 make sure we are calling SwerveDrive::Handle() in disabled
        std::cout << "orig x: " << m_PathData["waypoints"][0]["anchor"]["x"] << std::endl;
        std::cout << "orig y: " << m_PathData["waypoints"][0]["anchor"]["y"] << std::endl;
        std::cout << "pose x: " << curPose.X().value() << std::endl;
        std::cout << "pose y: " << curPose.Y().value() << std::endl;

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

void PathplannerSwerveCommand::Handle(CowRobot *robot)
{

    frc::Pose2d currentPose = robot->GetDrivetrain()->GetPose();

    pathplanner::PathPlannerTrajectory::State targetState
        = m_Trajectory->sample(units::second_t{ m_Timer->Get() });

    // this is called in the PathPlanner implementation of this method, not sure why
    // targetState = targetState.reverse();

    CowLib::CowChassisSpeeds chassisSpeeds
        = CowLib::CowChassisSpeeds::FromWPI(m_HolonomicController->calculateRobotRelativeSpeeds(currentPose, targetState));
 

    robot->GetDrivetrain()->SetVelocity(chassisSpeeds, false);
}

void PathplannerSwerveCommand::Finish(CowRobot *robot)
{
    if (m_Stop)
    {
        robot->GetDrivetrain()->SetVelocity(0, 0, 0, true);
    }

    m_Timer->Stop();
}