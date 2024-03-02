#include "Vision.h"

Vision::Vision()
{
    m_TickCount = 0;
}

Vision::Sample Vision::GetRobotPose()
{
    std::vector<double> limelightValues = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumberArray("botpose_wpiblue", std::vector<double>(11));

    frc::Pose3d pose3d { units::meter_t {limelightValues[0]},
                         units::meter_t {limelightValues[1]},
                         units::meter_t {limelightValues[2]},
                         frc::Rotation3d(
                            units::radian_t{ limelightValues[3] },
                            units::radian_t{ limelightValues[4] },
                            units::radian_t{ limelightValues[5] }
                         )};

    Vision::Sample sample;
    sample.pose3d = pose3d;
    sample.totalLatency = units::millisecond_t{ limelightValues[6] };
    sample.tagCount = limelightValues[7];
    sample.tagSpan = limelightValues[7];
    sample.averageTagDistance = limelightValues[7];
    sample.averageTagArea = limelightValues[7];

    return sample;
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
    m_TickCount++;

    if (m_LEDState == LEDState::OFF)
    {
        LEDOff();
    }
    else if (m_LEDState == LEDState::BLINK_SLOW || m_LEDState == LEDState::BLINK_FAST)
    {
        if (m_LEDState == LEDState::BLINK_SLOW)
        {
            if(m_TickCount == 0)
            {
                LEDOn();
            }
            else if (m_TickCount == CONSTANT("BLINK_SLOW_INTERVAL"))
            {
                LEDOff();
            }
            else if ( m_TickCount > CONSTANT("BLINK_SLOW_INTERVAL") * 2)
            {
                m_TickCount = 0;
            }
        }
        else if (m_LEDState == LEDState::BLINK_FAST)
        {
            if(m_TickCount == 0)
            {
                LEDOn();
            }
            else if (m_TickCount == CONSTANT("BLINK_FAST_INTERVAL"))
            {
                LEDOff();
            }
            else if (m_TickCount > CONSTANT("BLINK_FAST_INTERVAL") * 2)
            {
                m_TickCount = 0;
            }
        }
    }
}