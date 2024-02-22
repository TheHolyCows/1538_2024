#include "Vision.h"

Vision::Vision()
{

}

Vision::PoseWithLatency Vision::GetRobotPose()
{
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumberArray("<botpose_wpiblue>",std::vector<double>(7));
    std::vector<double> limelightValues = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumberArray("<botpose_wpiblue>",std::vector<double>(6));

    frc::Pose2d pose2d { units::meter_t {limelightValues[0]},
                         units::meter_t {limelightValues[1]},
                         units::degree_t {limelightValues[5]} }; 	

    double totalLatency = limelightValues[6];

    Vision::PoseWithLatency poseWithLatency;
    poseWithLatency.pose2d = pose2d;
    poseWithLatency.totalLatency = totalLatency;

    return poseWithLatency;
}

void Vision::SetLEDState(Vision::LEDState ledState)
{
    m_LEDState = ledState;
}

void Vision::Handle()
{
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", 4);



    if (m_LEDState == LEDState::OFF)
    {
        nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("force off", 1);
    }
    else if (m_LEDState == LEDState::BLINK_SLOW || m_LEDState == LEDState::BLINK_FAST)
    {
        double m_StateChangeTime = frc::Timer::GetFPGATimestamp().value();
        double timeElapsed = frc::Timer::GetFPGATimestamp().value() - m_StateChangeTime;
        
        if (m_LEDState == LEDState::BLINK_SLOW)
        {
            if (timeElapsed >= CONSTANT("BLINK_SLOW_TIME"))
            {
                // flip LED boolean
            }
            
        }
        else if (m_LEDState == LEDState::BLINK_FAST)
        {
            if (timeElapsed >= CONSTANT("BLINK_FAST_TIME"))
            {
                // flip LED boolean
            }
        }
    }
}