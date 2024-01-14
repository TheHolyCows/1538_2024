#include "VisionAlignCommand.h"

VisionAlignCommand::VisionAlignCommand(const double timeout, const ARM_CARGO cargo)
    : m_Timer(std::make_unique<CowLib::CowTimer>()),
      m_Gyro(*CowPigeon::GetInstance()),
      m_Timeout(timeout),
      m_Cargo(cargo)
{
}

bool VisionAlignCommand::IsComplete(CowRobot *robot)
{
    if (m_Timer->HasElapsed(m_Timeout))
    {
        robot->GetDrivetrain()->SetVelocity(0, 0, 0, true);
        return true;
    }

    //     Vision::GetInstance()->CubeYAligned()

    return false;
}

void VisionAlignCommand::Start(CowRobot *robot)
{
    m_Timer->Start();
}

void VisionAlignCommand::Handle(CowRobot *robot)
{
    robot->GetDriveController()->LockHeading(0, 0);
}

void VisionAlignCommand::Finish(CowRobot *robot)
{
    m_Timer->Stop();
}
