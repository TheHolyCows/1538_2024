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
                             bool overrideInitPose = false,
                             units::feet_per_second_t speed         = 20.21_fps,
                             units::feet_per_second_squared_t accel = 14_fps_sq)
    {
        std::deque<RobotCommand *> series;
        for (const auto &event : events)
        {
            series.push_back(new WaitCommand(event.delay, false));
            series.push_back(event.command);
        }

        return new ParallelCommand({ new PathplannerSwerveCommand(pathName, speed, accel, true, overrideInitPose),
                                     new SeriesCommand(series) });
    };


    /**
     * START AUTO MODE DEFS BELOW
    */

    /* test drive */
    // m_Modes["drive test"].push_back(new UpdateArmCommand(CONSTANT("WRIST_STOW_SETPOINT"),
    //                                     CONSTANT("PIVOT_STOW_SETPOINT"),
    //                                     false));
    // m_Modes["drive test"].push_back(pathWithEvents("drive3-1",
    //                                             { { 1.5_s, new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
    //                                                                             CONSTANT("PIVOT_GROUND_SETPOINT"),
    //                                                                             false) } },
    //                                             true,
    //                                             14_fps,
    //                                             8_fps_sq));
    // m_Modes["drive test"].push_back(pathWithEvents("drive3-2",
    //                                             { { 0.2_s, new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, false) },
    //                                               { 0.7_s, new UpdateArmCommand(CONSTANT("WRIST_STOW_SETPOINT"),
    //                                                                             CONSTANT("PIVOT_STOW_SETPOINT"),
    //                                                                             false) },
    //                                               { 0.5_s, new UpdateShooterStateCommand(Shooter::ShooterState::SPIN_UP, false) }},
    //                                             false,
    //                                             14_fps,
    //                                             8_fps_sq));
    // m_Modes["drive test"].push_back(new WaitCommand(0.3_s,false));
    // m_Modes["drive test"].push_back(new StationaryVisionCommand(1.0_s));
    // m_Modes["drive test"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    // m_Modes["drive test"].push_back(new WaitCommand(1_s,false));
    // m_Modes["drive test"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));
    // m_Modes["drive test"].push_back(new UpdateShooterStateCommand(Shooter::ShooterState::IDLE, false));
    // PathplannerSwerveCommand("drive3-1", 14_fps, 8_fps_sq, true, true));
    // m_Modes["drive test"].push_back(new PathplannerSwerveCommand("drive1-2", 6_fps, 8_fps_sq, true, false));

    // currently this will just follow a path - need to implement pulling a target from vison
    // should probably pass in a vision target to constructor
    // m_Modes["drive test"].push_back(new PathplannerVisionCommand("drive1-2", 6_fps, 8_fps_sq, 30.0, 70.0, true, false));


    /* test subsystems */
    m_Modes["subsys test"].push_back(new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
                                        CONSTANT("PIVOT_GROUND_SETPOINT"),
                                        false));
    m_Modes["subsys test"].push_back(new RaceCommand(
                                        { new WaitCommand(3_s,false),
                                          new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, true)
                                        }));
    m_Modes["subsys test"].push_back(new UpdateArmCommand(CONSTANT("WRIST_LAUNCH_SETPOINT"), CONSTANT("PIVOT_LAUNCH_SETPOINT"), true));
    m_Modes["subsys test"].push_back(new UpdateShooterStateCommand(Shooter::ShooterState::SPIN_UP, false));
    m_Modes["subsys test"].push_back(new StationaryVisionCommand(1.5_s));
    m_Modes["subsys test"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    m_Modes["subsys test"].push_back(new WaitCommand(3_s,false));
    m_Modes["subsys test"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));
    m_Modes["subsys test"].push_back(new UpdateShooterStateCommand(Shooter::ShooterState::IDLE, false));


    /* START 2024 AUTOS */
    /* [5] red amp -> amp far */
    // piece 1
    m_Modes["[5] red amp -> amp far"].push_back(new ParallelCommand(
                                        { new UpdateArmCommand(10, 80, false),
                                          new UpdateShooterStateCommand(Shooter::ShooterState::SPIN_UP, false)
                                        }
    ));
    m_Modes["[5] red amp -> amp far"].push_back(new WaitCommand(0.7_s,false));
    m_Modes["[5] red amp -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    m_Modes["[5] red amp -> amp far"].push_back(new WaitCommand(0.15_s,false));
    m_Modes["[5] red amp -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));
    // m_Modes["[5] red amp -> amp far"].push_back(new UpdateShooterStateCommand(Shooter::ShooterState::IDLE, false));
    m_Modes["[5] red amp -> amp far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
                                                        CONSTANT("PIVOT_GROUND_SETPOINT"),
                                                        false));
    // piece 2
    m_Modes["[5] red amp -> amp far"].push_back(pathWithEvents("red-amp_start-root",
                                                { { 0.01_s, new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, false) },
                                                  { 0.8_s, new UpdateArmCommand(CONSTANT("WRIST_LAUNCH_SETPOINT"),
                                                                                CONSTANT("PIVOT_LAUNCH_SETPOINT"),
                                                                                false) }}, //,
                                                //   { 0.3_s, new UpdateShooterStateCommand(Shooter::ShooterState::SPIN_UP, false) }},
                                                false,
                                                14_fps,
                                                8_fps_sq));
    m_Modes["[5] red amp -> amp far"].push_back(new StationaryVisionCommand(0.3_s));
    m_Modes["[5] red amp -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    m_Modes["[5] red amp -> amp far"].push_back(new WaitCommand(0.15_s,false));
    m_Modes["[5] red amp -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));
    // m_Modes["[5] red amp -> amp far"].push_back(new UpdateShooterStateCommand(Shooter::ShooterState::IDLE, false));
    m_Modes["[5] red amp -> amp far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_STOW_SETPOINT"),
                                                        CONSTANT("PIVOT_STOW_SETPOINT"),
                                                        false));
    // piece 3
    m_Modes["[5] red amp -> amp far"].push_back(pathWithEvents("red-amp_far-1",
                                                { { 1.0_s, new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
                                                                              CONSTANT("PIVOT_GROUND_SETPOINT"),
                                                                              false) },
                                                  { 0.01_s, new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, false) },
                                                  { 1.0_s, new UpdateArmCommand(CONSTANT("WRIST_LAUNCH_SETPOINT"),
                                                                                CONSTANT("PIVOT_LAUNCH_SETPOINT"),
                                                                                false) } },
                                                false,
                                                20_fps,
                                                12_fps_sq));
    m_Modes["[5] red amp -> amp far"].push_back(new StationaryVisionCommand(0.3_s));
    m_Modes["[5] red amp -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    m_Modes["[5] red amp -> amp far"].push_back(new WaitCommand(0.15_s,false));
    m_Modes["[5] red amp -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));
    m_Modes["[5] red amp -> amp far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_STOW_SETPOINT"),
                                                        CONSTANT("PIVOT_STOW_SETPOINT"),
                                                        false));

    // piece 4
    m_Modes["[5] red amp -> amp far"].push_back(pathWithEvents("red-amp_far-2",
                                                { { 0.6_s, new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
                                                                              CONSTANT("PIVOT_GROUND_SETPOINT"),
                                                                              false) },
                                                  { 0.01_s, new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, false) },
                                                  { 1.2_s, new UpdateArmCommand(CONSTANT("WRIST_LAUNCH_SETPOINT"),
                                                                                CONSTANT("PIVOT_LAUNCH_SETPOINT"),
                                                                                false) } },
                                                false,
                                                20_fps,
                                                12_fps_sq));
    m_Modes["[5] red amp -> amp far"].push_back(new StationaryVisionCommand(0.3_s));
    m_Modes["[5] red amp -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    m_Modes["[5] red amp -> amp far"].push_back(new WaitCommand(0.15_s,false));
    m_Modes["[5] red amp -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));
    m_Modes["[5] red amp -> amp far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_STOW_SETPOINT"),
                                                        CONSTANT("PIVOT_STOW_SETPOINT"),
                                                        false));

    // piece 5
    m_Modes["[5] red amp -> amp far"].push_back(pathWithEvents("red-amp_far-3",
                                                { { 1.0_s, new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
                                                                              CONSTANT("PIVOT_GROUND_SETPOINT"),
                                                                              false) },
                                                  { 0.01_s, new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, false) },
                                                  { 1.3_s, new UpdateArmCommand(CONSTANT("WRIST_LAUNCH_SETPOINT"),
                                                                                CONSTANT("PIVOT_LAUNCH_SETPOINT"),
                                                                                false) } },
                                                false,
                                                20_fps,
                                                12_fps_sq));
    m_Modes["[5] red amp -> amp far"].push_back(new StationaryVisionCommand(0.5_s));
    m_Modes["[5] red amp -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    m_Modes["[5] red amp -> amp far"].push_back(new WaitCommand(0.15_s,false));
    m_Modes["[5] red amp -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));

    // end
    m_Modes["[5] red amp -> amp far"].push_back(new UpdateShooterStateCommand(Shooter::ShooterState::IDLE, false));
    m_Modes["[5] red amp -> amp far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_STOW_SETPOINT"),
                                                        CONSTANT("PIVOT_STOW_SETPOINT"),
                                                        false));

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
