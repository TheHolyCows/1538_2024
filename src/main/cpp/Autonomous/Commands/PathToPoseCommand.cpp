#include "PathToPoseCommand.h"

PathToPoseCommand::PathToPoseCommand(units::second_t time, frc::Pose2d targetPose, bool stopAtEnd, bool resetOdometry)
{
    m_Timer = new CowLib::CowTimer();
    m_Stop  = stopAtEnd;
    m_TargetPose = targetPose;
    m_ResetOdometry = resetOdometry;

    m_XController        = new frc::PIDController(CONSTANT("AUTO_DRIVE_P"), // needs to be higher than pathplanner PID
                                                  CONSTANT("AUTO_DRIVE_I"),
                                                  CONSTANT("AUTO_DRIVE_D"));
    m_YController        = new frc::PIDController(CONSTANT("AUTO_DRIVE_P"),
                                                  CONSTANT("AUTO_DRIVE_I"),
                                                  CONSTANT("AUTO_DRIVE_D"));
    m_RotationController = new frc::PIDController(CONSTANT("AUTO_ROTATION_P"),
                                                  CONSTANT("AUTO_ROTATION_I"),
                                                  CONSTANT("AUTO_ROTATION_D"));

    m_TotalTime = time.value();
}

PathToPoseCommand::~PathToPoseCommand()
{
    delete m_Timer;
    delete m_RotationController;
}

bool PathToPoseCommand::IsComplete(CowRobot *robot)
{
    return m_Timer->HasElapsed(m_TotalTime);
}

void PathToPoseCommand::Start(CowRobot *robot)
{
    if (m_ResetOdometry)
    {
        robot->GetDrivetrain()->ResetOdometry(frc::Pose2d{ 0_ft, 0_ft, 0_deg });
    }

    m_Timer->Reset();
    m_Timer->Start();
}

void PathToPoseCommand::Handle(CowRobot *robot)
{
    frc::Pose2d currentPose = robot->GetDrivetrain()->GetPose();
    // CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG,
    //                           "current: x %f y %f angle %f",
    //                           currentPose.X().value(),
    //                           currentPose.Y().value(),
    //                           currentPose.Rotation().Degrees().value());

    double vx    = m_XController->Calculate(currentPose.X().convert<units::foot>().value(),
                                            m_TargetPose.X().convert<units::foot>().value());
    double vy    = m_YController->Calculate(currentPose.Y().convert<units::foot>().value(),
                                            m_TargetPose.Y().convert<units::foot>().value());
    double omega = m_RotationController->Calculate(currentPose.Rotation().Degrees().value(),
                                                   m_TargetPose.Rotation().Degrees().value());


    auto chassisSpeeds = CowLib::CowChassisSpeeds{ vx, vy, omega };

    // CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG,
    //                           "set to:  x %f y %f angle %f",
    //                           chassisSpeeds.vx,
    //                           chassisSpeeds.vy,
    //                           chassisSpeeds.omega);

    robot->GetDrivetrain()->SetVelocity(chassisSpeeds, false);
}

void PathToPoseCommand::Finish(CowRobot *robot)
{
    if (m_Stop)
    {
        robot->GetDrivetrain()->SetVelocity(0, 0, 0, true);
    }

    m_Timer->Stop();
}
