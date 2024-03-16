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
                            units::degree_t{ limelightValues[3] },
                            units::degree_t{ limelightValues[4] },
                            units::degree_t{ limelightValues[5] }
                         )};

    Vision::Sample sample;
    sample.pose3d = pose3d;
    sample.totalLatency = units::millisecond_t{ limelightValues[6] };
    sample.tagCount = limelightValues[7];
    sample.tagSpan = limelightValues[8];
    sample.averageTagDistance = limelightValues[9];
    sample.averageTagArea = limelightValues[10];

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

double Vision::GetTargetDist(std::optional<frc::DriverStation::Alliance> alliance, frc::Pose2d lookaheadPose)
{
    // Pivot and wrist targetting
    double robotX = lookaheadPose.X().convert<units::foot>().value();
    double robotY = lookaheadPose.Y().convert<units::foot>().value();

    frc::Translation2d targetXY = GetTargetXY(alliance);

    // this originally got the distance to the goal regardless of offsets, now it uses offsets
    // not sure if that is correct
    double dist = sqrtf(powf(targetXY.Y().value() - robotY, 2) + powf(targetXY.X().value() - robotX, 2));
    
    return dist;
}

frc::Translation2d Vision::GetTargetXY(std::optional<frc::DriverStation::Alliance> alliance)
{
    // maybe swap this to look at which half of the field we're on
    if (alliance.has_value())
    {
        if (alliance.value() == frc::DriverStation::Alliance::kRed)
        {
            return { RED_SPEAKER.X() - units::foot_t(CONSTANT("RED_GOAL_X_OFFSET")), 
                     RED_SPEAKER.Y() - units::foot_t(CONSTANT("RED_GOAL_Y_OFFSET"))};
            
        }
        else
        {
            return { BLUE_SPEAKER.X() - units::foot_t(CONSTANT("BLUE_GOAL_X_OFFSET")), 
                     BLUE_SPEAKER.Y() - units::foot_t(CONSTANT("BLUE_GOAL_Y_OFFSET"))};
        }
        
    }

    return { 0_ft, 0_ft };
}

void Vision::Handle()
{
    if (m_LEDState == LEDState::OFF)
    {
        LEDOff();
    }
    else if (m_LEDState == LEDState::BLINK_SLOW || m_LEDState == LEDState::BLINK_FAST)
    {
        if (m_LEDState == LEDState::BLINK_SLOW)
        {
            if(m_TickCount == 1)
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
            if(m_TickCount == 1)
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

    m_TickCount++;
}