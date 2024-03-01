#include "Vision.h"

Vision::Vision()
{
    m_TickCount = 0;
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

void Vision::LEDOn()
{
   nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", 3);
    m_IsLEDOn = true; 
}

void Vision::LEDOff()
{
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", 1);
    m_IsLEDOn = false;
}

void Vision::Handle()
{
//  nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", 4);
    m_TickCount++;

    if (m_LEDState == LEDState::OFF)
    {
        LEDOff();
    }
    else if (m_LEDState == LEDState::BLINK_SLOW || m_LEDState == LEDState::BLINK_FAST)
    {
        if (m_LEDState == LEDState::BLINK_SLOW)
        {
            if(m_TickCount < CONSTANT("BLINK_SLOW_INTERVAL"))
            {
                LEDOn();
            }
            else if (m_TickCount > CONSTANT("BLINK_SLOW_INTERVAL") && m_TickCount < CONSTANT("BLINK_SLOW_INTERVAL") * 2)
            {
                LEDOff();
            }
            else 
            {
                m_TickCount = 0;
            }
        }
        else if (m_LEDState == LEDState::BLINK_FAST)
        {
            if(m_TickCount < CONSTANT("BLINK_FAST_INTERVAL"))
            {
                LEDOn();
            }
            else if (m_TickCount > CONSTANT("BLINK_FAST_INTERVAL") && m_TickCount < CONSTANT("BLINK_FAST_INTERVAL") * 2)
            {
                LEDOff();
            }
            else 
            {
                m_TickCount = 0;
            }
        }
    }
}