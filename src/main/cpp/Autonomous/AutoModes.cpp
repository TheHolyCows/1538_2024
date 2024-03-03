#include "AutoModes.h"

AutoModes *AutoModes::s_Instance = nullptr;

AutoModes::AutoModes()
{   
    /**
     * These top two defenitions enable us to easily implement actions while following a path
     * they can be used within an auto like so:
     * m_Modes["example"].push_back(pathWithEvents("L3 Link LZ - intake cube",
                                { { 0.02_s, new UpdateArmStateCommand(ARM_STOW, CG_CUBE, false, false) },
                                  { 0.15_s, new ClawCommand(CLAW_INTAKE, 0) },
                                  { 0.01_s, new UpdateArmStateCommand(ARM_GND, CG_CUBE, false) } },
                                true,
                                20.21_fps,
                                11_fps_sq));
    */
    struct TrajectoryEvent
    {
        // IMPORTANT: Delays are from the previous event NOT the start of the path
        units::second_t delay;
        RobotCommand *command;
    };

    auto pathWithEvents = [](const std::string &pathName,
                             const std::vector<TrajectoryEvent> &events,
                             bool resetOdometry                     = false,
                             units::feet_per_second_t speed         = 20.21_fps,
                             units::feet_per_second_squared_t accel = 14_fps_sq)
    {
        std::deque<RobotCommand *> series;
        for (const auto &event : events)
        {
            series.push_back(new WaitCommand(event.delay, false));
            series.push_back(event.command);
        }

        return new ParallelCommand({ new PathplannerSwerveTrajectoryCommand(pathName, speed, accel, true, resetOdometry),
                                     new SeriesCommand(series) });
    };


    /**
     * START AUTO MODE DEFS BELOW
    */
    // m_Modes["testing"].push_back(new PathplannerSwerveTrajectoryCommand("drive1-1", 6_fps, 8_fps_sq, true, true));
    // m_Modes["testing"].push_back(new PathplannerSwerveTrajectoryCommand("drive1-2", 6_fps, 8_fps_sq, true, false));

    m_Modes["subsys test"].push_back(new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
                                        CONSTANT("PIVOT_GROUND_SETPOINT"),
                                        false));
    m_Modes["subsys test"].push_back(new RaceCommand(
                                        { new WaitCommand(3_s,false),
                                          new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_BEGIN, true)
                                        }));
    m_Modes["subsys test"].push_back(new UpdateArmCommand(CONSTANT("WRIST_LAUNCH_SETPOINT"), CONSTANT("PIVOT_LAUNCH_SETPOINT"), true));
    m_Modes["subsys test"].push_back(new UpdateShooterStateCommand(Shooter::ShooterState::SPIN_UP, false));
    m_Modes["subsys test"].push_back(new StationaryVisionCommand(1.5_s));
    m_Modes["subsys test"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));



    // Initialize auto mode selector
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
