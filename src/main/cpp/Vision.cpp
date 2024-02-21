#include "Vision.h"

Vision::Vision()
{

}

frc::Pose2d Vision::GetRobotPose()
{
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumberArray("<botpose_wpiblue>",std::vector<double>(6));
}

void Vision::SetLEDState()
{
    
}

