#include "PathplannerSwerveTrajectoryCommand.h"

PathplannerSwerveTrajectoryCommand::PathplannerSwerveTrajectoryCommand(const std::string &trajectoryName,
                                                                       units::feet_per_second_t maxVelocity,
                                                                       units::feet_per_second_squared_t maxAccel,
                                                                       frc::Rotation2d startingRotation,
                                                                       bool stopAtEnd,
                                                                       bool resetOdometry,
                                                                       std::vector<Event> events)
{
    // This is to make sure that it is loading trajectories on start and not on demand
    m_Timer         = new CowLib::CowTimer();
    m_Stop          = stopAtEnd;
    m_ResetOdometry = resetOdometry;


    // Load path from file
    m_Path = pathplanner::PathPlannerPath::fromPathFile(trajectoryName);

    // read in path file and modify velocity and acceleration based on values passed to this function
    const std::string filePath = frc::filesystem::GetDeployDirectory()
			+ "/pathplanner/paths/" + trajectoryName + ".path";
    std::ifstream pathFile(filePath);
    wpi::json data = wpi::json::parse(pathFile);
    pathFile.close();

    data["globalConstraints"]["maxVelocity"] = units::meters_per_second_t(maxVelocity).value();
    data["globalConstraints"]["maxAcceleration"] = units::meters_per_second_squared_t(maxAccel).value();


    // does not work - should use hot reload???
    // std::ofstream outFile(filePath);
    // outFile << data;
    // outFile.close();

    m_Path->hotReload(data);

    // get poses and rotations for start and end
    auto start_pose = data["waypoints"][0]["anchor"];
    auto end_pose = data["waypoints"][data["waypoints"].size() - 1]["anchor"];

    units::degree_t start_rot = 0_deg;
    std::cout << data["previewStartingState"] << std::endl;
    if (!data["previewStartingState"].is_null())
    {
        start_rot = units::degree_t(data["previewStartingState"]["rotation"]); // TODO: check NULL?
    }
    
    units::degree_t end_rot = units::degree_t(data["goalEndState"]["rotation"]);

    m_StartRotation = frc::Rotation2d(start_rot);
    m_StartPose = frc::Pose2d(units::meter_t(start_pose["x"]),units::meter_t(start_pose["y"]),start_rot);

    m_EndRotation = frc::Rotation2d(start_rot + end_rot);
    m_StartPose = frc::Pose2d(units::meter_t(end_pose["x"]),units::meter_t(end_pose["y"]),start_rot);

    m_Events = events;

    // CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "Loaded trajectory %s", trajectoryName.c_str());

    m_HolonomicController = new pathplanner::PPHolonomicDriveController(
        pathplanner::PIDConstants{ CONSTANT("AUTO_DRIVE_P"), CONSTANT("AUTO_DRIVE_I"), CONSTANT("AUTO_DRIVE_D") },
        pathplanner::PIDConstants{ CONSTANT("AUTO_ROTATION_P"), CONSTANT("AUTO_ROTATION_I"), CONSTANT("AUTO_ROTATION_D") },
        units::meters_per_second_t(18_fps),
        units::meter_t(units::length::foot_t(CONSTANT("AUTO_DRIVE_BASE_RADIUS"))),
        0.01_s);
    m_HolonomicController->setEnabled(true);

    frc::ChassisSpeeds startingSpeeds = frc::ChassisSpeeds { 0_mps, 0_mps, 0_rad_per_s};

    // need pose/speeds at end of last path to create trajectory, may need to pass in from constructor if compute time is bad
    m_Trajectory = std::make_shared<pathplanner::CowLibTrajectory>(m_Path,
                                                                        startingSpeeds,
                                                                        m_StartRotation);
}

PathplannerSwerveTrajectoryCommand::~PathplannerSwerveTrajectoryCommand()
{
    delete m_Timer;
    delete m_HolonomicController;
}

bool PathplannerSwerveTrajectoryCommand::IsComplete(CowRobot *robot)
{
    return m_Timer->HasElapsed(m_TotalTime);
}

void PathplannerSwerveTrajectoryCommand::Start(CowRobot *robot)
{
    CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG,"starting new path");

    // m_Trajectory = std::make_shared<CowLibTrajectory>(m_Path,
    //                                                                     robot->GetDrivetrain()->GetChassisSpeeds(),
    //                                                                     frc::Rotation2d(units::degree_t(robot->GetDrivetrain()->GetPoseRot())));
    
    if (m_ResetOdometry)
    {
        frc::Pose2d curPose = robot->GetDrivetrain()->GetPose();
        robot->GetDrivetrain()->ResetOdometry(m_StartPose);
        CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG,"cur pose: x:%lf y:%lf rot:%lf",curPose.X(), curPose.Y(), curPose.Rotation().Degrees());
        CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG,"target pose: x:%lf y:%lf rot:%lf",m_StartPose.X(), m_StartPose.Y(), m_StartPose.Rotation().Degrees());
    }

    m_TotalTime = m_Trajectory->getTotalTime().value();

    // std::vector<pathplanner::PathPlannerTrajectory::EventMarker> markers = m_Trajectory.getMarkers();

    // for (Event event : m_Events)
    // {
    //     for (pathplanner::PathPlannerTrajectory::EventMarker marker : markers)
    //     {
    //         if (std::find(marker.names.begin(), marker.names.end(), event.waypointName) != marker.names.end())
    //         {
    //             event.time = marker.time.value();
    //         }
    //     }
    // }

    m_Timer->Reset();
    m_Timer->Start();
}

void PathplannerSwerveTrajectoryCommand::Handle(CowRobot *robot)
{
    // for (Event event : m_Events)
    // {
    //     if (m_Timer->HasPeriodPassed(event.time) && !event.done)
    //     {
    //         if (!event.started)
    //         {
    //             event.command->Start(robot);
    //             event.started = true;
    //         }

    //         if (event.command->IsComplete(robot))
    //         {
    //             event.done = true;
    //             event.command->Finish(robot);
    //         }
    //         else
    //         {
    //             event.command->Handle(robot);
    //         }
    //     }
    // }

    frc::Pose2d currentPose = robot->GetDrivetrain()->GetPose();

    pathplanner::PathPlannerTrajectory::State targetState
        = m_Trajectory->sample(units::second_t{ m_Timer->Get() });
    // targetState = targetState.reverse();
    
    // CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG,"cur pose: x:%lf y:%lf rot:%lf",currentPose.X(), currentPose.Y(), currentPose.Rotation().Degrees());
    // CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG,"target pose: x:%lf y:%lf rot:%lf",targetState.getTargetHolonomicPose().X(), targetState.getTargetHolonomicPose().Y(), targetState.getTargetHolonomicPose().Rotation().Degrees());

    CowLib::CowChassisSpeeds chassisSpeeds
        = CowLib::CowChassisSpeeds::FromWPI(m_HolonomicController->calculateRobotRelativeSpeeds(currentPose, targetState));
    // CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG,"target speeds: x:%lf y:%lf rot:%lf",chassisSpeeds.vx, chassisSpeeds.vy, chassisSpeeds.omega);
 

    // auto err = frc::Transform2d(currentPose, targetState.pose);
    // frc::SmartDashboard::PutNumber("auto/swerve/error/x", err.Translation().X().convert<units::foot>().value());
    // frc::SmartDashboard::PutNumber("auto/swerve/error/y", err.Translation().Y().convert<units::foot>().value());
    // frc::SmartDashboard::PutNumber("auto/swerve/error/rotation", err.Rotation().Degrees().value());
    // frc::SmartDashboard::PutNumber("auto/swerve/actual/x",
    //                                currentPose.Translation().X().convert<units::foot>().value());
    // frc::SmartDashboard::PutNumber("auto/swerve/actual/y",
    //                                currentPose.Translation().Y().convert<units::foot>().value());
    // frc::SmartDashboard::PutNumber("auto/swerve/actual/rotation", currentPose.Rotation().Degrees().value());
    // frc::SmartDashboard::PutNumber("auto/swerve/target/x",
    //                                targetState.asWPILibState().pose.Translation().X().convert<units::foot>().value());
    // frc::SmartDashboard::PutNumber("auto/swerve/target/y",
    //                                targetState.asWPILibState().pose.Translation().Y().convert<units::foot>().value());
    // frc::SmartDashboard::PutNumber("auto/swerve/target/rotation",
    //                                targetState.asWPILibState().pose.Rotation().Degrees().value());

    robot->GetDrivetrain()->SetVelocity(chassisSpeeds, false);
}

void PathplannerSwerveTrajectoryCommand::Finish(CowRobot *robot)
{
    if (m_Stop)
    {
        CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "Stopping swerve trajectory command");
        robot->GetDrivetrain()->SetVelocity(0, 0, 0, true);
    }

    for (Event event : m_Events)
    {
        if (!event.done && event.started)
        {
            event.command->Finish(robot);
        }
    }

    m_Timer->Stop();
}

frc::Pose2d PathplannerSwerveTrajectoryCommand::GetStartingPose()
{
    return m_Path->getPreviewStartingHolonomicPose();
    // return m_Trajectory.getInitialHolonomicPose();
}

frc::Rotation2d PathplannerSwerveTrajectoryCommand::GetEndRot()
{
    return m_EndRotation;
}