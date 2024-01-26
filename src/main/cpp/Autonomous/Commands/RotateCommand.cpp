#include "RotateCommand.h"

RotateCommand::RotateCommand(units::second_t time, units::degree_t targetRotation, bool stopAtEnd, bool resetOdometry)
{
    m_Timer = new CowLib::CowTimer();
    m_Stop  = stopAtEnd;
    m_TargetRotation = targetRotation;
    m_ResetOdometry = resetOdometry;

    m_RotationController = new frc::PIDController(CONSTANT("AUTO_ROTATION_P"),
                                                   CONSTANT("AUTO_ROTATION_I"),
                                                   CONSTANT("AUTO_ROTATION_D"));

    m_TotalTime = time.value();
}

RotateCommand::~RotateCommand()
{
    delete m_Timer;
    delete m_RotationController;
}

bool RotateCommand::IsComplete(CowRobot *robot)
{
    return m_Timer->HasElapsed(m_TotalTime);
}

void RotateCommand::Start(CowRobot *robot)
{
    if (m_ResetOdometry)
    {
        robot->GetDrivetrain()->ResetOdometry(frc::Pose2d{ 0_ft, 0_ft, 0_deg });
    }

    m_Pose = robot->GetDrivetrain()->GetPose();
    m_TargetPose = frc::Pose2d{m_Pose.X(),m_Pose.Y(),m_TargetRotation};

    m_Timer->Reset();
    m_Timer->Start();
}

void RotateCommand::Handle(CowRobot *robot)
{
    frc::Pose2d currentPose = robot->GetDrivetrain()->GetPose();
    // CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG,
    //                           "current: x %f y %f angle %f",
    //                           currentPose.X().value(),
    //                           currentPose.Y().value(),
    //                           currentPose.Rotation().Degrees().value());

    double vx    = 0;
    double vy    = 0;
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

void RotateCommand::Finish(CowRobot *robot)
{
    if (m_Stop)
    {
        robot->GetDrivetrain()->SetVelocity(0, 0, 0, true);
    }

    m_Timer->Stop();
}
