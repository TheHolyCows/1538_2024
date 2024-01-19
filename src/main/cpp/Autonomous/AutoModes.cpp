#include "AutoModes.h"

AutoModes *AutoModes::s_Instance = nullptr;

AutoModes::AutoModes()
{
    struct TrajectoryEvent
    {
        // Important: Delays are from the previous event NOT the start of the path
        double delay;
        RobotCommand *command;
    };

    auto pathWithEvents = [](const std::string &name,
                             const std::vector<TrajectoryEvent> &events,
                             bool resetOdometry = true,
                             units::feet_per_second_t speed       = 20.21_fps,
                             units::feet_per_second_squared_t accel       = 14_fps_sq)
    {
        std::deque<RobotCommand *> series;
        for (const auto &event : events)
        {
            series.push_back(new WaitCommand(event.delay, false));
            series.push_back(event.command);
        }

        return new ParallelCommand({ new PathplannerSwerveTrajectoryCommand(name, speed, accel, frc::Rotation2d(0_deg), true, resetOdometry),
                                     new SeriesCommand(series) });
    };

    m_Modes["testing"].push_back(new PathplannerSwerveTrajectoryCommand("drive1-1",6_fps,8_fps_sq,frc::Rotation2d(0_deg),true,true));
    m_Modes["testing"].push_back(new PathplannerSwerveTrajectoryCommand("drive1-2", 6_fps, 8_fps_sq,
                                                                        frc::Rotation2d(0_deg),
                                                                        true, false));


    m_Iterator = m_Modes.begin();
}

AutoModes::~AutoModes()
{
    for (auto &mode : m_Modes)
    {
        for (auto command : mode.second)
        {
            delete command;
        }
    }
}

AutoModes *AutoModes::GetInstance()
{
    if (s_Instance == nullptr)
    {
        s_Instance = new AutoModes();
    }

    return s_Instance;
}

std::deque<RobotCommand *> AutoModes::GetCommandList()
{
    return m_Iterator->second;
}

std::string AutoModes::GetName()
{
    return m_Iterator->first;
}

void AutoModes::NextMode()
{
    ++m_Iterator;

    if (m_Iterator == m_Modes.end())
    {
        m_Iterator = m_Modes.begin();
    }

    // Display the name of the current auto mode to driver station
    std::string name = GetName();
    std::cout << "Auto mode: " << name << std::endl;
}
