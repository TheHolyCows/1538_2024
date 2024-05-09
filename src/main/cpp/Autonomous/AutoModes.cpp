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

    auto preloadCommand = [](double preloadDist, double nextShotDist)
    {
        return new SeriesCommand(
        {
            new UpdateArmCommand(-10, 80, false),
            new UpdateShooterSpeed(preloadDist),
            new UpdateShooterStateCommand(Shooter::ShooterState::SPIN_UP, false),
            new WaitCommand(0.5_s,false), // time to release pin
            new UpdateArmCommand(preloadDist,
                                 CONSTANT("PIVOT_LAUNCH_SETPOINT"),
                                 false,
                                 true),
            new StationaryVisionCommand(1.35_s),
            new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false),
            new WaitCommand(0.20_s,false),
            new UpdateShooterStateCommand(Shooter::ShooterState::IDLE, false),
            new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false),
        });
    };

    auto skipPreloadCommand = [](double preloadDist, double nextShotDist)
    {
        return new SeriesCommand(
        {
            new UpdateArmCommand(5, 80, false),
            new UpdateShooterSpeed(nextShotDist),
            new WaitCommand(0.5_s,false), // time to release pin
            new UpdateArmCommand(CONSTANT("WRIST_STOW_SETPOINT"),
                                 CONSTANT("PIVOT_STOW_SETPOINT"),
                                 false),
            new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false)
        });
    };

    auto visionPathWithEvents = [](const std::string &pathName,
                             double startOverride,
                             double endOverride,
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

        return new ParallelCommand({ new PathplannerVisionCommand(pathName, speed, accel, startOverride,
                                                                  endOverride, true, overrideInitPose),
                                     new SeriesCommand(series) });
    };


    /**
     * START AUTO MODE DEFS BELOW
    */

    m_Modes["pit test"].push_back(preloadCommand(7.82, 8.044));
    m_Modes["pit test"].push_back(new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
                                                        CONSTANT("PIVOT_GROUND_SETPOINT"),
                                                        false));
    m_Modes["pit test"].push_back(new WaitCommand(0.6_s,false));





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
    // m_Modes["subsys test"].push_back(new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
    //                                     CONSTANT("PIVOT_GROUND_SETPOINT"),
    //                                     false));
    // m_Modes["subsys test"].push_back(new RaceCommand(
    //                                     { new WaitCommand(3_s,false),
    //                                       new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, true)
    //                                     }));
    // m_Modes["subsys test"].push_back(new UpdateArmCommand(CONSTANT("WRIST_LAUNCH_SETPOINT"), CONSTANT("PIVOT_LAUNCH_SETPOINT"), true));
    // m_Modes["subsys test"].push_back(new UpdateShooterStateCommand(Shooter::ShooterState::SPIN_UP, false));
    // m_Modes["subsys test"].push_back(new StationaryVisionCommand(1.5_s));
    // m_Modes["subsys test"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    // m_Modes["subsys test"].push_back(new WaitCommand(3_s,false));
    // m_Modes["subsys test"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));
    // m_Modes["subsys test"].push_back(new UpdateShooterStateCommand(Shooter::ShooterState::IDLE, false));


    /* START 2024 AUTOS */
    // should have:
    //   x red amp to amp far 4 piece - tuned
    //   x red amp to amp far (skip near) 4 piece - tuning
    //   x red source to amp far 5 piece - tuned
    //   x red source to source far 3/4 piece
    //   x red source to mid far (mid first -> source) 3/4 piece
    //   - optional drop for last 3? (skip preload)
    // then same for blue

    /* all paths */
    // [4] red amp -> amp far: red-amp_start-root -> red-amp_far-1 -> red-amp_far-2
    // [3] red amp no preload -> amp far: red-amp-drop-start -> red-amp-drop-1 -> red-amp-drop-2
    // [5] red source -> amp far: red-source_get-close_start -> red_get-close_2 -> red_get-close_3 -> red-amp_far-1_from-shoot
    // [3] red source -> source far: red-source-get-far_start -> red-source-get_far-1 -> red-source-get_far-2
    // [3] red source -> mid far: red-source-drop-start -> red-source-get_far-1 -> red-source_get-mid-source_from-shoot



    /* [4] red amp -> amp far: */
    // paths: red-amp_start-root -> red-amp_far-1 -> red-amp_far-2
    // pre-load
    m_Modes["[4] red amp -> amp far"].push_back(preloadCommand(5.65, 12.89));
    m_Modes["[4] red amp -> amp far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
                                                        CONSTANT("PIVOT_GROUND_SETPOINT"),
                                                        false));
    m_Modes["[4] red amp -> amp far"].push_back(new WaitCommand(0.5_s,false));
    // piece 2
    m_Modes["[4] red amp -> amp far"].push_back(pathWithEvents("red-amp_start-root",
                                                { { 0.01_s, new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, false) },
                                                  { 0.9_s, new UpdateArmCommand(12.89,  // based on distance of 12.89 above - TODO: update to use distance and read from map
                                                                                CONSTANT("PIVOT_LAUNCH_SETPOINT"),
                                                                                false,
                                                                                true) },
                                                  { 0.01_s, new UpdateShooterStateCommand(Shooter::ShooterState::SPIN_UP, false) }},
                                                true,
                                                12_fps,
                                                8_fps_sq));
    m_Modes["[4] red amp -> amp far"].push_back(new StationaryVisionCommand(0.7_s));
    m_Modes["[4] red amp -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    m_Modes["[4] red amp -> amp far"].push_back(new WaitCommand(0.15_s,false));
    m_Modes["[4] red amp -> amp far"].push_back(new UpdateShooterStateCommand(Shooter::ShooterState::IDLE, false));
    m_Modes["[4] red amp -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));
    // m_Modes["[4] red amp -> amp far"].push_back(new UpdateShooterSpeed(18.54)); // distance for shot 3 and 4 in feet
    m_Modes["[4] red amp -> amp far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
                                                            CONSTANT("PIVOT_GROUND_SETPOINT"),
                                                            false));
    m_Modes["[4] red amp -> amp far"].push_back(new WaitCommand(0.2_s,false));
    // piece 3
    m_Modes["[4] red amp -> amp far"].push_back(pathWithEvents("red-amp_far-1",
                                                  { { 0.01_s, new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
                                                                                    CONSTANT("PIVOT_GROUND_SETPOINT"),
                                                                                    false) },
                                                    { 0.2_s, new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, false) },
                                                    { 1.7_s, new UpdateArmCommand(18.54, // based on distance of 14.16 above
                                                                                CONSTANT("PIVOT_LAUNCH_SETPOINT"),
                                                                                false,
                                                                                true) },
                                                    { 0.01_s, new UpdateShooterStateCommand(Shooter::ShooterState::SPIN_UP, false) }},
                                                false,
                                                16_fps,
                                                15_fps_sq));
    m_Modes["[4] red amp -> amp far"].push_back(new StationaryVisionCommand(0.5_s));
    m_Modes["[4] red amp -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    m_Modes["[4] red amp -> amp far"].push_back(new WaitCommand(0.25_s,false));
    m_Modes["[4] red amp -> amp far"].push_back(new UpdateShooterStateCommand(Shooter::ShooterState::IDLE, false));
    m_Modes["[4] red amp -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));
    m_Modes["[4] red amp -> amp far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_STOW_SETPOINT"),
                                                        CONSTANT("PIVOT_STOW_SETPOINT"),
                                                        false));

    // piece 4
    m_Modes["[4] red amp -> amp far"].push_back(pathWithEvents("red-amp_far-2",
                                                { { 0.4_s, new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
                                                                              CONSTANT("PIVOT_GROUND_SETPOINT"),
                                                                              false) },
                                                  { 0.01_s, new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, false) },
                                                  { 1.3_s, new UpdateArmCommand(18.54, // based on distance of 14.16 above
                                                                                CONSTANT("PIVOT_LAUNCH_SETPOINT"),
                                                                                false,
                                                                                true) },
                                                  { 0.01_s, new UpdateShooterStateCommand(Shooter::ShooterState::SPIN_UP, false) }},
                                                false,
                                                16_fps,
                                                15_fps_sq));
    m_Modes["[4] red amp -> amp far"].push_back(new StationaryVisionCommand(0.5_s));
    m_Modes["[4] red amp -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    m_Modes["[4] red amp -> amp far"].push_back(new WaitCommand(0.15_s,false));

    // end
    m_Modes["[4] red amp -> amp far"].push_back(new UpdateShooterStateCommand(Shooter::ShooterState::IDLE, false));
    m_Modes["[4] red amp -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));
    m_Modes["[4] red amp -> amp far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_STOW_SETPOINT"),
                                                        CONSTANT("PIVOT_STOW_SETPOINT"),
                                                        false));

    /* [5] !force! red amp -> amp far */
    // pre-load
    // m_Modes["[5] !force! red amp -> amp far"].push_back(new ParallelCommand(
    //                                     { new UpdateArmCommand(10, 80, false),
    //                                       new UpdateShooterSpeed(5.65) // 15 is lowest dist value in shooter range dist map
    //                                     }
    // ));
    // m_Modes["[5] !force! red amp -> amp far"].push_back(new UpdateShooterStateCommand(Shooter::ShooterState::SPIN_UP, false));
    // m_Modes["[5] !force! red amp -> amp far"].push_back(new WaitCommand(0.5_s,false));
    // m_Modes["[5] !force! red amp -> amp far"].push_back(new UpdateArmCommand(5.65,
    //                                                   CONSTANT("PIVOT_LAUNCH_SETPOINT"),
    //                                                   false,
    //                                                   true));
    // m_Modes["[5] !force! red amp -> amp far"].push_back(new StationaryVisionCommand(0.5_s));
    // m_Modes["[5] !force! red amp -> amp far"].push_back(new WaitCommand(0.8_s,false));
    // m_Modes["[5] !force! red amp -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    // m_Modes["[5] !force! red amp -> amp far"].push_back(new WaitCommand(0.15_s,false));
    // m_Modes["[5] !force! red amp -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));
    // m_Modes["[5] !force! red amp -> amp far"].push_back(new UpdateShooterSpeed(12.89)); // distance for shot 2 in feet
    // m_Modes["[5] !force! red amp -> amp far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
    //                                                     CONSTANT("PIVOT_GROUND_SETPOINT"),
    //                                                     false));
    // m_Modes["[5] !force! red amp -> amp far"].push_back(new WaitCommand(0.6_s,false));
    // // piece 2
    // m_Modes["[5] !force! red amp -> amp far"].push_back(pathWithEvents("red-amp_start-root",
    //                                             { { 0.01_s, new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, false) },
    //                                               { 0.9_s, new UpdateArmCommand(12.89,  // based on distance of 12.89 above - TODO: update to use distance and read from map
    //                                                                             CONSTANT("PIVOT_LAUNCH_SETPOINT"),
    //                                                                             false,
    //                                                                             true) }},
    //                                             //   { 0.3_s, new UpdateShooterStateCommand(Shooter::ShooterState::SPIN_UP, false) }},
    //                                             false,
    //                                             12_fps,
    //                                             8_fps_sq));
    // m_Modes["[5] !force! red amp -> amp far"].push_back(new StationaryVisionCommand(0.5_s));
    // m_Modes["[5] !force! red amp -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    // m_Modes["[5] !force! red amp -> amp far"].push_back(new WaitCommand(0.15_s,false));
    // m_Modes["[5] !force! red amp -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));
    // m_Modes["[5] !force! red amp -> amp far"].push_back(new UpdateShooterSpeed(14.16)); // distance for shot 3 and 4 in feet
    // m_Modes["[5] !force! red amp -> amp far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
    //                                                         CONSTANT("PIVOT_GROUND_SETPOINT"),
    //                                                         false));
    // m_Modes["[5] !force! red amp -> amp far"].push_back(new WaitCommand(0.6_s,false));
    // // piece 3
    // m_Modes["[5] !force! red amp -> amp far"].push_back(pathWithEvents("red-amp_far-1",
    //                                               { { 0.01_s, new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
    //                                                                                 CONSTANT("PIVOT_GROUND_SETPOINT"),
    //                                                                                 false) },
    //                                                 { 0.2_s, new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, false) },
    //                                                 { 1.7_s, new UpdateArmCommand(14.16, // based on distance of 14.16 above
    //                                                                             CONSTANT("PIVOT_LAUNCH_SETPOINT"),
    //                                                                             false,
    //                                                                             true) } },
    //                                             false,
    //                                             21.5_fps,
    //                                             14.5_fps_sq));
    // m_Modes["[5] !force! red amp -> amp far"].push_back(new StationaryVisionCommand(0.5_s));
    // m_Modes["[5] !force! red amp -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    // m_Modes["[5] !force! red amp -> amp far"].push_back(new WaitCommand(0.15_s,false));
    // m_Modes["[5] !force! red amp -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));
    // m_Modes["[5] !force! red amp -> amp far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_STOW_SETPOINT"),
    //                                                     CONSTANT("PIVOT_STOW_SETPOINT"),
    //                                                     false));

    // // piece 4
    // m_Modes["[5] !force! red amp -> amp far"].push_back(pathWithEvents("red-amp_far-2",
    //                                             { { 0.4_s, new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
    //                                                                           CONSTANT("PIVOT_GROUND_SETPOINT"),
    //                                                                           false) },
    //                                               { 0.01_s, new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, false) },
    //                                               { 1.0_s, new UpdateArmCommand(14.16, // based on distance of 14.16 above
    //                                                                             CONSTANT("PIVOT_LAUNCH_SETPOINT"),
    //                                                                             false,
    //                                                                             true) } },
    //                                             false,
    //                                             21.5_fps,
    //                                             14.5_fps_sq));
    // m_Modes["[5] !force! red amp -> amp far"].push_back(new StationaryVisionCommand(0.5_s));
    // m_Modes["[5] !force! red amp -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    // m_Modes["[5] !force! red amp -> amp far"].push_back(new WaitCommand(0.15_s,false));

    // // end
    // m_Modes["[5] !force! red amp -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));
    // m_Modes["[5] !force! red amp -> amp far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_STOW_SETPOINT"),
    //                                                     CONSTANT("PIVOT_STOW_SETPOINT"),
    //                                                     false));



    /* [3] red amp no preload -> amp far */
    // paths: red-amp-drop-start -> red-amp-drop-1 -> red-amp-drop-2

    // pre-load
    m_Modes["[3] red amp no preload -> amp far"].push_back(skipPreloadCommand(5.65, 14.16));

    // piece 1
    m_Modes["[3] red amp no preload -> amp far"].push_back(pathWithEvents("red-amp-drop-start",
                                                { { 1.0_s, new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
                                                                                CONSTANT("PIVOT_GROUND_SETPOINT"),
                                                                                false) },
                                                  { 0.01_s, new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, false) },
                                                  { 3.0_s, new UpdateArmCommand(14.16,
                                                                                CONSTANT("PIVOT_LAUNCH_SETPOINT"),
                                                                                false,
                                                                                true) },
                                                  { 0.01_s, new UpdateShooterStateCommand(Shooter::ShooterState::SPIN_UP, false) }},
                                                true,
                                                18_fps,
                                                18_fps_sq));
    m_Modes["[3] red amp no preload -> amp far"].push_back(new StationaryVisionCommand(0.5_s));
    m_Modes["[3] red amp no preload -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    m_Modes["[3] red amp no preload -> amp far"].push_back(new WaitCommand(0.15_s,false));
    m_Modes["[3] red amp no preload -> amp far"].push_back(new UpdateShooterStateCommand(Shooter::ShooterState::IDLE, false));
    m_Modes["[3] red amp no preload -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));
    m_Modes["[3] red amp no preload -> amp far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_STOW_SETPOINT"),
                                                        CONSTANT("PIVOT_STOW_SETPOINT"),
                                                        false));

    // piece 2
    m_Modes["[3] red amp no preload -> amp far"].push_back(pathWithEvents("red-amp-drop-1",
                                                  { { 0.01_s, new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
                                                                                    CONSTANT("PIVOT_GROUND_SETPOINT"),
                                                                                    false) },
                                                    { 0.2_s, new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, false) },
                                                    { 1.8_s, new UpdateArmCommand(14.16, // based on distance of 14.16 above
                                                                                CONSTANT("PIVOT_LAUNCH_SETPOINT"),
                                                                                false,
                                                                                true) },
                                                  { 0.01_s, new UpdateShooterStateCommand(Shooter::ShooterState::SPIN_UP, false) }},
                                                false,
                                                18_fps,
                                                18_fps_sq));
    m_Modes["[3] red amp no preload -> amp far"].push_back(new StationaryVisionCommand(0.5_s));
    m_Modes["[3] red amp no preload -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    m_Modes["[3] red amp no preload -> amp far"].push_back(new WaitCommand(0.15_s,false));
    m_Modes["[3] red amp no preload -> amp far"].push_back(new UpdateShooterStateCommand(Shooter::ShooterState::IDLE, false));
    m_Modes["[3] red amp no preload -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));
    m_Modes["[3] red amp no preload -> amp far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_STOW_SETPOINT"),
                                                        CONSTANT("PIVOT_STOW_SETPOINT"),
                                                        false));

    // piece 3
    m_Modes["[3] red amp no preload -> amp far"].push_back(pathWithEvents("red-amp-drop-2",
                                                { { 0.4_s, new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
                                                                              CONSTANT("PIVOT_GROUND_SETPOINT"),
                                                                              false) },
                                                  { 0.01_s, new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, false) },
                                                  { 1.8_s, new UpdateArmCommand(14.16, // based on distance of 14.16 above
                                                                                CONSTANT("PIVOT_LAUNCH_SETPOINT"),
                                                                                false,
                                                                                true) },
                                                  { 0.01_s, new UpdateShooterStateCommand(Shooter::ShooterState::SPIN_UP, false) }},
                                                false,
                                                14_fps,
                                                14_fps_sq));
    m_Modes["[3] red amp no preload -> amp far"].push_back(new StationaryVisionCommand(0.5_s));
    m_Modes["[3] red amp no preload -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    m_Modes["[3] red amp no preload -> amp far"].push_back(new WaitCommand(0.15_s,false));

    // end
    m_Modes["[3] red amp no preload -> amp far"].push_back(new UpdateShooterStateCommand(Shooter::ShooterState::IDLE, false));
    m_Modes["[3] red amp no preload -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));
    m_Modes["[3] red amp no preload -> amp far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_STOW_SETPOINT"),
                                                        CONSTANT("PIVOT_STOW_SETPOINT"),
                                                        false));


    /* [5] red source -> amp far */
    // paths: red-source_get-close_start -> red_get-close_2 -> red_get-close_3 -> red-amp_far-1_from-shoot

    // pre-load
    m_Modes["[5] red source -> amp far"].push_back(preloadCommand(7.82, 8.044));
    m_Modes["[5] red source -> amp far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
                                                        CONSTANT("PIVOT_GROUND_SETPOINT"),
                                                        false));
    m_Modes["[5] red source -> amp far"].push_back(new WaitCommand(0.15_s,false));

    // piece 2
    m_Modes["[5] red source -> amp far"].push_back(pathWithEvents("red-source_get-close_start",
                                                { { 0.01_s, new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, false) },
                                                  { 1.15_s, new UpdateArmCommand(8.044,  // based on distance of 8.044
                                                                                CONSTANT("PIVOT_LAUNCH_SETPOINT"),
                                                                                false,
                                                                                true) },
                                                  { 0.01_s, new UpdateShooterStateCommand(Shooter::ShooterState::SPIN_UP, false) }},
                                                true,
                                                18_fps,
                                                14_fps_sq));
    m_Modes["[5] red source -> amp far"].push_back(new StationaryVisionCommand(0.9_s));
    m_Modes["[5] red source -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    m_Modes["[5] red source -> amp far"].push_back(new WaitCommand(0.15_s,false));
    m_Modes["[5] red source -> amp far"].push_back(new UpdateShooterStateCommand(Shooter::ShooterState::IDLE, false));
    m_Modes["[5] red source -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));
    // m_Modes["[5] red source -> amp far"].push_back(new UpdateShooterSpeed(7.28)); // distance for shot 3 in feet
    m_Modes["[5] red source -> amp far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
                                                            CONSTANT("PIVOT_GROUND_SETPOINT"),
                                                            false));
    m_Modes["[5] red source -> amp far"].push_back(new WaitCommand(0.15_s,false)); // not necessary?

    // piece 3
    m_Modes["[5] red source -> amp far"].push_back(pathWithEvents("red_get-close_2",
                                                { { 0.01_s, new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, false) },
                                                  { 1.5_s, new UpdateArmCommand(7.28,  // based on distance of 7.28
                                                                                CONSTANT("PIVOT_LAUNCH_SETPOINT"),
                                                                                false,
                                                                                true) },
                                                  { 0.01_s, new UpdateShooterStateCommand(Shooter::ShooterState::SPIN_UP, false) }},
                                                false,
                                                18_fps,
                                                14_fps_sq));
    m_Modes["[5] red source -> amp far"].push_back(new StationaryVisionCommand(0.9_s));
    m_Modes["[5] red source -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    m_Modes["[5] red source -> amp far"].push_back(new WaitCommand(0.15_s,false));
    m_Modes["[5] red source -> amp far"].push_back(new UpdateShooterStateCommand(Shooter::ShooterState::IDLE, false));
    m_Modes["[5] red source -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));
    // m_Modes["[5] red source -> amp far"].push_back(new UpdateShooterSpeed(14.16)); // distance for shot 4 in feet
    m_Modes["[5] red source -> amp far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
                                                            CONSTANT("PIVOT_GROUND_SETPOINT"),
                                                            false));

    // piece 4
    m_Modes["[5] red source -> amp far"].push_back(pathWithEvents("red_get-close_3",
                                                { { 0.2_s, new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, false) },
                                                  { 1.3_s, new UpdateArmCommand(14.16,  // based on distance of 14.16
                                                                                CONSTANT("PIVOT_LAUNCH_SETPOINT"),
                                                                                false,
                                                                                true) },
                                                  { 0.01_s, new UpdateShooterStateCommand(Shooter::ShooterState::SPIN_UP, false) }},
                                                false,
                                                14_fps,
                                                14_fps_sq));
    m_Modes["[5] red source -> amp far"].push_back(new StationaryVisionCommand(0.5_s));
    m_Modes["[5] red source -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    m_Modes["[5] red source -> amp far"].push_back(new WaitCommand(0.15_s,false));
    m_Modes["[5] red source -> amp far"].push_back(new UpdateShooterStateCommand(Shooter::ShooterState::IDLE, false));
    m_Modes["[5] red source -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));
    // m_Modes["[5] red source -> amp far"].push_back(new UpdateShooterSpeed(14.16)); // distance for shot 5 in feet
    m_Modes["[5] red source -> amp far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_STOW_SETPOINT"),
                                                            CONSTANT("PIVOT_STOW_SETPOINT"),
                                                            false));
    // piece 5
    m_Modes["[5] red source -> amp far"].push_back(pathWithEvents("red-amp_far-1_from-shoot",
                                                { { 0.5_s, new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
                                                                                    CONSTANT("PIVOT_GROUND_SETPOINT"),
                                                                                    false) },
                                                  { 0.2_s, new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, false) },
                                                  { 1.0_s, new UpdateArmCommand(14.16, // based on distance of 14.16 above
                                                                                CONSTANT("PIVOT_LAUNCH_SETPOINT"),
                                                                                false,
                                                                                true) },
                                                  { 0.01_s, new UpdateShooterStateCommand(Shooter::ShooterState::SPIN_UP, false) }},
                                                false,
                                                21.5_fps,
                                                14.5_fps_sq));
    m_Modes["[5] red source -> amp far"].push_back(new StationaryVisionCommand(0.5_s));
    m_Modes["[5] red source -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    m_Modes["[5] red source -> amp far"].push_back(new WaitCommand(0.15_s,false));
    
    // end
    m_Modes["[5] red source -> amp far"].push_back(new UpdateShooterStateCommand(Shooter::ShooterState::IDLE, false));
    m_Modes["[5] red source -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));
    m_Modes["[5] red source -> amp far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_STOW_SETPOINT"),
                                                        CONSTANT("PIVOT_STOW_SETPOINT"),
                                                        false));

    /* [3] red source -> source far */
    // paths: red-source-get-far_start -> red-source-get_far-1 -> red-source-get_far-2

    // pre-load
    m_Modes["[3] red source -> source far"].push_back(preloadCommand(7.82, 17.68));

    m_Modes["[3] red source -> source far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
                                                        CONSTANT("PIVOT_GROUND_SETPOINT"),
                                                        false));

    // piece 2
    m_Modes["[3] red source -> source far"].push_back(pathWithEvents("red-source-get-far_start",
                                                { { 0.01_s, new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, false) },
                                                  { 3.8_s, new UpdateArmCommand(17.68, // orig 3.0_s
                                                                                CONSTANT("PIVOT_LAUNCH_SETPOINT"),
                                                                                false,
                                                                                true) },
                                                  { 0.01_s, new UpdateShooterStateCommand(Shooter::ShooterState::SPIN_UP, false) }},
                                                true,
                                                14_fps,
                                                16_fps_sq));
    m_Modes["[3] red source -> source far"].push_back(new StationaryVisionCommand(0.6_s));
    m_Modes["[3] red source -> source far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    m_Modes["[3] red source -> source far"].push_back(new WaitCommand(0.15_s,false));
    m_Modes["[3] red source -> source far"].push_back(new UpdateShooterStateCommand(Shooter::ShooterState::IDLE, false));
    m_Modes["[3] red source -> source far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));

    // piece 3
    m_Modes["[3] red source -> source far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
                                                        CONSTANT("PIVOT_GROUND_SETPOINT"),
                                                        false));
    m_Modes["[3] red source -> source far"].push_back(pathWithEvents("red-source-get_far-1",
                                                { { 0.01_s, new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, false) },
                                                  { 3.3_s, new UpdateArmCommand(17.68,  // orig 2.4s
                                                                                CONSTANT("PIVOT_LAUNCH_SETPOINT"),
                                                                                false,
                                                                                true) },
                                                  { 0.01_s, new UpdateShooterStateCommand(Shooter::ShooterState::SPIN_UP, false) }},
                                                true,
                                                12_fps,
                                                14_fps_sq));
    m_Modes["[3] red source -> source far"].push_back(new StationaryVisionCommand(0.5_s));
    m_Modes["[3] red source -> source far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    m_Modes["[3] red source -> source far"].push_back(new WaitCommand(0.15_s,false));
    m_Modes["[3] red source -> source far"].push_back(new UpdateShooterStateCommand(Shooter::ShooterState::IDLE, false));
    m_Modes["[3] red source -> source far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));

    // piece 4
    // m_Modes["[4] red source -> source far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
    //                                                     CONSTANT("PIVOT_GROUND_SETPOINT"),
    //                                                     false));
    // m_Modes["[4] red source -> source far"].push_back(new WaitCommand(0.5_s,false)); // wait so we dont destroy arm
    // m_Modes["[4] red source -> source far"].push_back(pathWithEvents("red-source-get_far-2",
    //                                             { { 0.01_s, new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, false) },
    //                                               { 5.8_s, new UpdateArmCommand(17.68,
    //                                                                             CONSTANT("PIVOT_LAUNCH_SETPOINT"),
    //                                                                             false,
    //                                                                             true) }},
    //                                             true,
    //                                             4_fps,
    //                                             4_fps_sq));
    // m_Modes["[4] red source -> source far"].push_back(new StationaryVisionCommand(0.5_s));
    // m_Modes["[4] red source -> source far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    // m_Modes["[4] red source -> source far"].push_back(new WaitCommand(0.15_s,false));
    // m_Modes["[4] red source -> source far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));

    // end
    m_Modes["[3] red source -> source far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_STOW_SETPOINT"),
                                                            CONSTANT("PIVOT_STOW_SETPOINT"),
                                                            false));

    /* [3] red source -> mid far */
    // paths: red-source-drop-start -> red-source-get_far-1 -> red-source_get-mid-source_from-shoot

    // pre-load
    m_Modes["[3] red source -> mid far"].push_back(preloadCommand(7.82,17.68));
    m_Modes["[3] red source -> mid far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_STOW_SETPOINT"),
                                                        CONSTANT("PIVOT_STOW_SETPOINT"),
                                                        false));
    // m_Modes["[3] red source -> mid far"].push_back(new WaitCommand(0.3_s,false)); // wait so we dont destroy arm

    // piece 2
    m_Modes["[3] red source -> mid far"].push_back(pathWithEvents("red-source-drop-start",
                                                { { 2.15_s, new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"), // was 1.6
                                                                                CONSTANT("PIVOT_GROUND_SETPOINT"),
                                                                                false)},
                                                  { 0.01_s, new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, false) },
                                                  { 1.1_s, new UpdateArmCommand(CONSTANT("WRIST_STOW_SETPOINT"), // was 1
                                                                                CONSTANT("PIVOT_STOW_SETPOINT"),
                                                                                false)},
                                                  { 2.2_s, new UpdateArmCommand(17.68,  // was 1.5
                                                                                CONSTANT("PIVOT_LAUNCH_SETPOINT"),
                                                                                false,
                                                                                true) },
                                                  { 0.01_s, new UpdateShooterStateCommand(Shooter::ShooterState::SPIN_UP, false) }},
                                                true,
                                                12_fps,
                                                12_fps_sq));
    m_Modes["[3] red source -> mid far"].push_back(new StationaryVisionCommand(0.5_s));
    m_Modes["[3] red source -> mid far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    m_Modes["[3] red source -> mid far"].push_back(new WaitCommand(0.15_s,false));
    m_Modes["[3] red source -> mid far"].push_back(new UpdateShooterStateCommand(Shooter::ShooterState::IDLE, false));
    m_Modes["[3] red source -> mid far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));

    // piece 3
    m_Modes["[3] red source -> mid far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
                                                        CONSTANT("PIVOT_GROUND_SETPOINT"),
                                                        false));
    m_Modes["[3] red source -> mid far"].push_back(pathWithEvents("red-source-get_far-1",
                                                { { 0.01_s, new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, false) },
                                                  { 3.5_s, new UpdateArmCommand(17.68,
                                                                                CONSTANT("PIVOT_LAUNCH_SETPOINT"),
                                                                                false,
                                                                                true) },
                                                  { 0.01_s, new UpdateShooterStateCommand(Shooter::ShooterState::SPIN_UP, false) }},
                                                true,
                                                12_fps,
                                                8_fps_sq));
    m_Modes["[3] red source -> mid far"].push_back(new StationaryVisionCommand(0.5_s));
    m_Modes["[3] red source -> mid far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    m_Modes["[3] red source -> mid far"].push_back(new WaitCommand(0.15_s,false));
    m_Modes["[3] red source -> mid far"].push_back(new UpdateShooterStateCommand(Shooter::ShooterState::IDLE, false));
    m_Modes["[3] red source -> mid far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));

    // piece 4
    // m_Modes["[3] red source -> mid far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
    //                                                     CONSTANT("PIVOT_GROUND_SETPOINT"),
    //                                                     false));
    // m_Modes["[3] red source -> mid far"].push_back(pathWithEvents("red-source_get-mid-source_from-shoot",
    //                                             { { 0.01_s, new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, false) },
    //                                               { 2.3_s, new UpdateArmCommand(17.68,
    //                                                                             CONSTANT("PIVOT_LAUNCH_SETPOINT"),
    //                                                                             false,
    //                                                                             true) }},
    //                                             true,
    //                                             4_fps,
    //                                             4_fps_sq));
    // m_Modes["[3] red source -> mid far"].push_back(new StationaryVisionCommand(0.5_s));
    // m_Modes["[3] red source -> mid far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    // m_Modes["[3] red source -> mid far"].push_back(new WaitCommand(0.15_s,false));

    // end
    m_Modes["[3] red source -> mid far"].push_back(new UpdateShooterStateCommand(Shooter::ShooterState::IDLE, false));
    m_Modes["[3] red source -> mid far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));
    m_Modes["[3] red source -> mid far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_STOW_SETPOINT"),
                                                            CONSTANT("PIVOT_STOW_SETPOINT"),
                                                            false));


    /************/
    /*** BLUE ***/
    /************/

    /* [4] blue amp -> amp far: */
    // paths: blue-amp_start-root -> blue-amp_far-1 -> blue-amp_far-2
    // pre-load
    m_Modes["[4] blue amp -> amp far"].push_back(preloadCommand(5.65, 12.89));
    m_Modes["[4] blue amp -> amp far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
                                                        CONSTANT("PIVOT_GROUND_SETPOINT"),
                                                        false));
    m_Modes["[4] blue amp -> amp far"].push_back(new WaitCommand(0.5_s,false));
    // piece 2
    m_Modes["[4] blue amp -> amp far"].push_back(pathWithEvents("blue-amp_start-root",
                                                { { 0.01_s, new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, false) },
                                                  { 0.9_s, new UpdateArmCommand(12.89,  // based on distance of 12.89 above - TODO: update to use distance and read from map
                                                                                CONSTANT("PIVOT_LAUNCH_SETPOINT"),
                                                                                false,
                                                                                true) },
                                                  { 0.01_s, new UpdateShooterStateCommand(Shooter::ShooterState::SPIN_UP, false) }},
                                                true,
                                                12_fps,
                                                8_fps_sq));
    m_Modes["[4] blue amp -> amp far"].push_back(new StationaryVisionCommand(0.7_s));
    m_Modes["[4] blue amp -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    m_Modes["[4] blue amp -> amp far"].push_back(new WaitCommand(0.15_s,false));
    m_Modes["[4] blue amp -> amp far"].push_back(new UpdateShooterStateCommand(Shooter::ShooterState::IDLE, false));
    m_Modes["[4] blue amp -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));
    // m_Modes["[4] blue amp -> amp far"].push_back(new UpdateShooterSpeed(18.54)); // distance for shot 3 and 4 in feet
    m_Modes["[4] blue amp -> amp far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
                                                            CONSTANT("PIVOT_GROUND_SETPOINT"),
                                                            false));
    m_Modes["[4] blue amp -> amp far"].push_back(new WaitCommand(0.2_s,false));
    // piece 3
    m_Modes["[4] blue amp -> amp far"].push_back(pathWithEvents("blue-amp_far-1",
                                                  { { 0.01_s, new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
                                                                                    CONSTANT("PIVOT_GROUND_SETPOINT"),
                                                                                    false) },
                                                    { 0.2_s, new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, false) },
                                                    { 1.7_s, new UpdateArmCommand(18.54, // based on distance of 14.16 above
                                                                                CONSTANT("PIVOT_LAUNCH_SETPOINT"),
                                                                                false,
                                                                                true) },
                                                    { 0.01_s, new UpdateShooterStateCommand(Shooter::ShooterState::SPIN_UP, false) }},
                                                false,
                                                16_fps,
                                                15_fps_sq));
    m_Modes["[4] blue amp -> amp far"].push_back(new StationaryVisionCommand(0.5_s));
    m_Modes["[4] blue amp -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    m_Modes["[4] blue amp -> amp far"].push_back(new WaitCommand(0.25_s,false));
    m_Modes["[4] blue amp -> amp far"].push_back(new UpdateShooterStateCommand(Shooter::ShooterState::IDLE, false));
    m_Modes["[4] blue amp -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));
    m_Modes["[4] blue amp -> amp far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_STOW_SETPOINT"),
                                                        CONSTANT("PIVOT_STOW_SETPOINT"),
                                                        false));

    // piece 4
    m_Modes["[4] blue amp -> amp far"].push_back(pathWithEvents("blue-amp_far-2",
                                                { { 0.4_s, new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
                                                                              CONSTANT("PIVOT_GROUND_SETPOINT"),
                                                                              false) },
                                                  { 0.01_s, new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, false) },
                                                  { 1.3_s, new UpdateArmCommand(18.54, // based on distance of 14.16 above
                                                                                CONSTANT("PIVOT_LAUNCH_SETPOINT"),
                                                                                false,
                                                                                true) },
                                                  { 0.01_s, new UpdateShooterStateCommand(Shooter::ShooterState::SPIN_UP, false) }},
                                                false,
                                                16_fps,
                                                15_fps_sq));
    m_Modes["[4] blue amp -> amp far"].push_back(new StationaryVisionCommand(0.5_s));
    m_Modes["[4] blue amp -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    m_Modes["[4] blue amp -> amp far"].push_back(new WaitCommand(0.15_s,false));

    // end
    m_Modes["[4] blue amp -> amp far"].push_back(new UpdateShooterStateCommand(Shooter::ShooterState::IDLE, false));
    m_Modes["[4] blue amp -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));
    m_Modes["[4] blue amp -> amp far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_STOW_SETPOINT"),
                                                        CONSTANT("PIVOT_STOW_SETPOINT"),
                                                        false));

    /* [5] !force! blue amp -> amp far */
    // pre-load
    // m_Modes["[5] !force! blue amp -> amp far"].push_back(new ParallelCommand(
    //                                     { new UpdateArmCommand(10, 80, false),
    //                                       new UpdateShooterSpeed(5.65) // 15 is lowest dist value in shooter range dist map
    //                                     }
    // ));
    // m_Modes["[5] !force! blue amp -> amp far"].push_back(new UpdateShooterStateCommand(Shooter::ShooterState::SPIN_UP, false));
    // m_Modes["[5] !force! blue amp -> amp far"].push_back(new WaitCommand(0.5_s,false));
    // m_Modes["[5] !force! blue amp -> amp far"].push_back(new UpdateArmCommand(5.65,
    //                                                   CONSTANT("PIVOT_LAUNCH_SETPOINT"),
    //                                                   false,
    //                                                   true));
    // m_Modes["[5] !force! blue amp -> amp far"].push_back(new StationaryVisionCommand(0.5_s));
    // m_Modes["[5] !force! blue amp -> amp far"].push_back(new WaitCommand(0.8_s,false));
    // m_Modes["[5] !force! blue amp -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    // m_Modes["[5] !force! blue amp -> amp far"].push_back(new WaitCommand(0.15_s,false));
    // m_Modes["[5] !force! blue amp -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));
    // m_Modes["[5] !force! blue amp -> amp far"].push_back(new UpdateShooterSpeed(12.89)); // distance for shot 2 in feet
    // m_Modes["[5] !force! blue amp -> amp far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
    //                                                     CONSTANT("PIVOT_GROUND_SETPOINT"),
    //                                                     false));
    // m_Modes["[5] !force! blue amp -> amp far"].push_back(new WaitCommand(0.6_s,false));
    // // piece 2
    // m_Modes["[5] !force! blue amp -> amp far"].push_back(pathWithEvents("blue-amp_start-root",
    //                                             { { 0.01_s, new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, false) },
    //                                               { 0.9_s, new UpdateArmCommand(12.89,  // based on distance of 12.89 above - TODO: update to use distance and read from map
    //                                                                             CONSTANT("PIVOT_LAUNCH_SETPOINT"),
    //                                                                             false,
    //                                                                             true) }},
    //                                             //   { 0.3_s, new UpdateShooterStateCommand(Shooter::ShooterState::SPIN_UP, false) }},
    //                                             false,
    //                                             12_fps,
    //                                             8_fps_sq));
    // m_Modes["[5] !force! blue amp -> amp far"].push_back(new StationaryVisionCommand(0.5_s));
    // m_Modes["[5] !force! blue amp -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    // m_Modes["[5] !force! blue amp -> amp far"].push_back(new WaitCommand(0.15_s,false));
    // m_Modes["[5] !force! blue amp -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));
    // m_Modes["[5] !force! blue amp -> amp far"].push_back(new UpdateShooterSpeed(14.16)); // distance for shot 3 and 4 in feet
    // m_Modes["[5] !force! blue amp -> amp far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
    //                                                         CONSTANT("PIVOT_GROUND_SETPOINT"),
    //                                                         false));
    // m_Modes["[5] !force! blue amp -> amp far"].push_back(new WaitCommand(0.6_s,false));
    // // piece 3
    // m_Modes["[5] !force! blue amp -> amp far"].push_back(pathWithEvents("blue-amp_far-1",
    //                                               { { 0.01_s, new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
    //                                                                                 CONSTANT("PIVOT_GROUND_SETPOINT"),
    //                                                                                 false) },
    //                                                 { 0.2_s, new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, false) },
    //                                                 { 1.7_s, new UpdateArmCommand(14.16, // based on distance of 14.16 above
    //                                                                             CONSTANT("PIVOT_LAUNCH_SETPOINT"),
    //                                                                             false,
    //                                                                             true) } },
    //                                             false,
    //                                             21.5_fps,
    //                                             14.5_fps_sq));
    // m_Modes["[5] !force! blue amp -> amp far"].push_back(new StationaryVisionCommand(0.5_s));
    // m_Modes["[5] !force! blue amp -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    // m_Modes["[5] !force! blue amp -> amp far"].push_back(new WaitCommand(0.15_s,false));
    // m_Modes["[5] !force! blue amp -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));
    // m_Modes["[5] !force! blue amp -> amp far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_STOW_SETPOINT"),
    //                                                     CONSTANT("PIVOT_STOW_SETPOINT"),
    //                                                     false));

    // // piece 4
    // m_Modes["[5] !force! blue amp -> amp far"].push_back(pathWithEvents("blue-amp_far-2",
    //                                             { { 0.4_s, new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
    //                                                                           CONSTANT("PIVOT_GROUND_SETPOINT"),
    //                                                                           false) },
    //                                               { 0.01_s, new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, false) },
    //                                               { 1.0_s, new UpdateArmCommand(14.16, // based on distance of 14.16 above
    //                                                                             CONSTANT("PIVOT_LAUNCH_SETPOINT"),
    //                                                                             false,
    //                                                                             true) } },
    //                                             false,
    //                                             21.5_fps,
    //                                             14.5_fps_sq));
    // m_Modes["[5] !force! blue amp -> amp far"].push_back(new StationaryVisionCommand(0.5_s));
    // m_Modes["[5] !force! blue amp -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    // m_Modes["[5] !force! blue amp -> amp far"].push_back(new WaitCommand(0.15_s,false));

    // // end
    // m_Modes["[5] !force! blue amp -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));
    // m_Modes["[5] !force! blue amp -> amp far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_STOW_SETPOINT"),
    //                                                     CONSTANT("PIVOT_STOW_SETPOINT"),
    //                                                     false));



    /* [3] blue amp no preload -> amp far */
    // paths: blue-amp-drop-start -> blue-amp-drop-1 -> blue-amp-drop-2

    // pre-load
    m_Modes["[3] blue amp no preload -> amp far"].push_back(skipPreloadCommand(5.65, 14.16));

    // piece 1
    m_Modes["[3] blue amp no preload -> amp far"].push_back(pathWithEvents("blue-amp-drop-start",
                                                { { 1.0_s, new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
                                                                                CONSTANT("PIVOT_GROUND_SETPOINT"),
                                                                                false) },
                                                  { 0.01_s, new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, false) },
                                                  { 3.0_s, new UpdateArmCommand(14.16,
                                                                                CONSTANT("PIVOT_LAUNCH_SETPOINT"),
                                                                                false,
                                                                                true) },
                                                  { 0.01_s, new UpdateShooterStateCommand(Shooter::ShooterState::SPIN_UP, false) }},
                                                true,
                                                18_fps,
                                                18_fps_sq));
    m_Modes["[3] blue amp no preload -> amp far"].push_back(new StationaryVisionCommand(0.5_s));
    m_Modes["[3] blue amp no preload -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    m_Modes["[3] blue amp no preload -> amp far"].push_back(new WaitCommand(0.15_s,false));
    m_Modes["[3] blue amp no preload -> amp far"].push_back(new UpdateShooterStateCommand(Shooter::ShooterState::IDLE, false));
    m_Modes["[3] blue amp no preload -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));
    m_Modes["[3] blue amp no preload -> amp far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_STOW_SETPOINT"),
                                                        CONSTANT("PIVOT_STOW_SETPOINT"),
                                                        false));

    // piece 2
    m_Modes["[3] blue amp no preload -> amp far"].push_back(pathWithEvents("blue-amp-drop-1",
                                                  { { 0.01_s, new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
                                                                                    CONSTANT("PIVOT_GROUND_SETPOINT"),
                                                                                    false) },
                                                    { 0.2_s, new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, false) },
                                                    { 1.8_s, new UpdateArmCommand(14.16, // based on distance of 14.16 above
                                                                                CONSTANT("PIVOT_LAUNCH_SETPOINT"),
                                                                                false,
                                                                                true) },
                                                  { 0.01_s, new UpdateShooterStateCommand(Shooter::ShooterState::SPIN_UP, false) }},
                                                false,
                                                18_fps,
                                                18_fps_sq));
    m_Modes["[3] blue amp no preload -> amp far"].push_back(new StationaryVisionCommand(0.5_s));
    m_Modes["[3] blue amp no preload -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    m_Modes["[3] blue amp no preload -> amp far"].push_back(new WaitCommand(0.15_s,false));
    m_Modes["[3] blue amp no preload -> amp far"].push_back(new UpdateShooterStateCommand(Shooter::ShooterState::IDLE, false));
    m_Modes["[3] blue amp no preload -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));
    m_Modes["[3] blue amp no preload -> amp far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_STOW_SETPOINT"),
                                                        CONSTANT("PIVOT_STOW_SETPOINT"),
                                                        false));

    // piece 3
    m_Modes["[3] blue amp no preload -> amp far"].push_back(pathWithEvents("blue-amp-drop-2",
                                                { { 0.4_s, new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
                                                                              CONSTANT("PIVOT_GROUND_SETPOINT"),
                                                                              false) },
                                                  { 0.01_s, new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, false) },
                                                  { 1.8_s, new UpdateArmCommand(14.16, // based on distance of 14.16 above
                                                                                CONSTANT("PIVOT_LAUNCH_SETPOINT"),
                                                                                false,
                                                                                true) },
                                                  { 0.01_s, new UpdateShooterStateCommand(Shooter::ShooterState::SPIN_UP, false) }},
                                                false,
                                                14_fps,
                                                14_fps_sq));
    m_Modes["[3] blue amp no preload -> amp far"].push_back(new StationaryVisionCommand(0.5_s));
    m_Modes["[3] blue amp no preload -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    m_Modes["[3] blue amp no preload -> amp far"].push_back(new WaitCommand(0.15_s,false));

    // end
    m_Modes["[3] blue amp no preload -> amp far"].push_back(new UpdateShooterStateCommand(Shooter::ShooterState::IDLE, false));
    m_Modes["[3] blue amp no preload -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));
    m_Modes["[3] blue amp no preload -> amp far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_STOW_SETPOINT"),
                                                        CONSTANT("PIVOT_STOW_SETPOINT"),
                                                        false));


    /* [5] blue source -> amp far */
    // paths: blue-source_get-close_start -> blue_get-close_2 -> blue_get-close_3 -> blue-amp_far-1_from-shoot

    // pre-load
    m_Modes["[5] blue source -> amp far"].push_back(preloadCommand(7.82, 8.044));
    m_Modes["[5] blue source -> amp far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
                                                        CONSTANT("PIVOT_GROUND_SETPOINT"),
                                                        false));
    m_Modes["[5] blue source -> amp far"].push_back(new WaitCommand(0.15_s,false));

    // piece 2
    m_Modes["[5] blue source -> amp far"].push_back(pathWithEvents("blue-source_get-close_start",
                                                { { 0.01_s, new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, false) },
                                                  { 1.15_s, new UpdateArmCommand(8.044,  // based on distance of 8.044
                                                                                CONSTANT("PIVOT_LAUNCH_SETPOINT"),
                                                                                false,
                                                                                true) },
                                                  { 0.01_s, new UpdateShooterStateCommand(Shooter::ShooterState::SPIN_UP, false) }},
                                                true,
                                                18_fps,
                                                14_fps_sq));
    m_Modes["[5] blue source -> amp far"].push_back(new StationaryVisionCommand(0.9_s));
    m_Modes["[5] blue source -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    m_Modes["[5] blue source -> amp far"].push_back(new WaitCommand(0.15_s,false));
    m_Modes["[5] blue source -> amp far"].push_back(new UpdateShooterStateCommand(Shooter::ShooterState::IDLE, false));
    m_Modes["[5] blue source -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));
    // m_Modes["[5] blue source -> amp far"].push_back(new UpdateShooterSpeed(7.28)); // distance for shot 3 in feet
    m_Modes["[5] blue source -> amp far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
                                                            CONSTANT("PIVOT_GROUND_SETPOINT"),
                                                            false));
    m_Modes["[5] blue source -> amp far"].push_back(new WaitCommand(0.15_s,false)); // not necessary?

    // piece 3
    m_Modes["[5] blue source -> amp far"].push_back(pathWithEvents("blue_get-close_2",
                                                { { 0.01_s, new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, false) },
                                                  { 1.5_s, new UpdateArmCommand(7.28,  // based on distance of 7.28
                                                                                CONSTANT("PIVOT_LAUNCH_SETPOINT"),
                                                                                false,
                                                                                true) },
                                                  { 0.01_s, new UpdateShooterStateCommand(Shooter::ShooterState::SPIN_UP, false) }},
                                                false,
                                                18_fps,
                                                14_fps_sq));
    m_Modes["[5] blue source -> amp far"].push_back(new StationaryVisionCommand(0.9_s));
    m_Modes["[5] blue source -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    m_Modes["[5] blue source -> amp far"].push_back(new WaitCommand(0.15_s,false));
    m_Modes["[5] blue source -> amp far"].push_back(new UpdateShooterStateCommand(Shooter::ShooterState::IDLE, false));
    m_Modes["[5] blue source -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));
    // m_Modes["[5] blue source -> amp far"].push_back(new UpdateShooterSpeed(14.16)); // distance for shot 4 in feet
    m_Modes["[5] blue source -> amp far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
                                                            CONSTANT("PIVOT_GROUND_SETPOINT"),
                                                            false));

    // piece 4
    m_Modes["[5] blue source -> amp far"].push_back(pathWithEvents("blue_get-close_3",
                                                { { 0.2_s, new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, false) },
                                                  { 1.3_s, new UpdateArmCommand(14.16,  // based on distance of 14.16
                                                                                CONSTANT("PIVOT_LAUNCH_SETPOINT"),
                                                                                false,
                                                                                true) },
                                                  { 0.01_s, new UpdateShooterStateCommand(Shooter::ShooterState::SPIN_UP, false) }},
                                                false,
                                                14_fps,
                                                14_fps_sq));
    m_Modes["[5] blue source -> amp far"].push_back(new StationaryVisionCommand(0.5_s));
    m_Modes["[5] blue source -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    m_Modes["[5] blue source -> amp far"].push_back(new WaitCommand(0.15_s,false));
    m_Modes["[5] blue source -> amp far"].push_back(new UpdateShooterStateCommand(Shooter::ShooterState::IDLE, false));
    m_Modes["[5] blue source -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));
    // m_Modes["[5] blue source -> amp far"].push_back(new UpdateShooterSpeed(14.16)); // distance for shot 5 in feet
    m_Modes["[5] blue source -> amp far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_STOW_SETPOINT"),
                                                            CONSTANT("PIVOT_STOW_SETPOINT"),
                                                            false));
    // piece 5
    m_Modes["[5] blue source -> amp far"].push_back(pathWithEvents("blue-amp_far-1_from-shoot",
                                                { { 0.5_s, new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
                                                                                    CONSTANT("PIVOT_GROUND_SETPOINT"),
                                                                                    false) },
                                                  { 0.2_s, new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, false) },
                                                  { 1.0_s, new UpdateArmCommand(14.16, // based on distance of 14.16 above
                                                                                CONSTANT("PIVOT_LAUNCH_SETPOINT"),
                                                                                false,
                                                                                true) },
                                                  { 0.01_s, new UpdateShooterStateCommand(Shooter::ShooterState::SPIN_UP, false) }},
                                                false,
                                                21.5_fps,
                                                14.5_fps_sq));
    m_Modes["[5] blue source -> amp far"].push_back(new StationaryVisionCommand(0.5_s));
    m_Modes["[5] blue source -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    m_Modes["[5] blue source -> amp far"].push_back(new WaitCommand(0.15_s,false));
    
    // end
    m_Modes["[5] blue source -> amp far"].push_back(new UpdateShooterStateCommand(Shooter::ShooterState::IDLE, false));
    m_Modes["[5] blue source -> amp far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));
    m_Modes["[5] blue source -> amp far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_STOW_SETPOINT"),
                                                        CONSTANT("PIVOT_STOW_SETPOINT"),
                                                        false));

    /* [3] blue source -> source far */
    // paths: blue-source-get-far_start -> blue-source-get_far-1 -> blue-source-get_far-2

    // pre-load
    m_Modes["[3] blue source -> source far"].push_back(preloadCommand(7.82, 17.68));

    m_Modes["[3] blue source -> source far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
                                                        CONSTANT("PIVOT_GROUND_SETPOINT"),
                                                        false));

    // piece 2
    m_Modes["[3] blue source -> source far"].push_back(pathWithEvents("blue-source-get-far_start",
                                                { { 0.01_s, new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, false) },
                                                  { 3.8_s, new UpdateArmCommand(17.68, // orig 3.0_s
                                                                                CONSTANT("PIVOT_LAUNCH_SETPOINT"),
                                                                                false,
                                                                                true) },
                                                  { 0.01_s, new UpdateShooterStateCommand(Shooter::ShooterState::SPIN_UP, false) }},
                                                true,
                                                14_fps,
                                                16_fps_sq));
    m_Modes["[3] blue source -> source far"].push_back(new StationaryVisionCommand(0.6_s));
    m_Modes["[3] blue source -> source far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    m_Modes["[3] blue source -> source far"].push_back(new WaitCommand(0.15_s,false));
    m_Modes["[3] blue source -> source far"].push_back(new UpdateShooterStateCommand(Shooter::ShooterState::IDLE, false));
    m_Modes["[3] blue source -> source far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));

    // piece 3
    m_Modes["[3] blue source -> source far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
                                                        CONSTANT("PIVOT_GROUND_SETPOINT"),
                                                        false));
    m_Modes["[3] blue source -> source far"].push_back(pathWithEvents("blue-source-get_far-1",
                                                { { 0.01_s, new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, false) },
                                                  { 3.3_s, new UpdateArmCommand(17.68,  // orig 2.4s
                                                                                CONSTANT("PIVOT_LAUNCH_SETPOINT"),
                                                                                false,
                                                                                true) },
                                                  { 0.01_s, new UpdateShooterStateCommand(Shooter::ShooterState::SPIN_UP, false) }},
                                                true,
                                                12_fps,
                                                14_fps_sq));
    m_Modes["[3] blue source -> source far"].push_back(new StationaryVisionCommand(0.5_s));
    m_Modes["[3] blue source -> source far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    m_Modes["[3] blue source -> source far"].push_back(new WaitCommand(0.15_s,false));
    m_Modes["[3] blue source -> source far"].push_back(new UpdateShooterStateCommand(Shooter::ShooterState::IDLE, false));
    m_Modes["[3] blue source -> source far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));

    // piece 4
    // m_Modes["[4] blue source -> source far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
    //                                                     CONSTANT("PIVOT_GROUND_SETPOINT"),
    //                                                     false));
    // m_Modes["[4] blue source -> source far"].push_back(new WaitCommand(0.5_s,false)); // wait so we dont destroy arm
    // m_Modes["[4] blue source -> source far"].push_back(pathWithEvents("blue-source-get_far-2",
    //                                             { { 0.01_s, new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, false) },
    //                                               { 5.8_s, new UpdateArmCommand(17.68,
    //                                                                             CONSTANT("PIVOT_LAUNCH_SETPOINT"),
    //                                                                             false,
    //                                                                             true) }},
    //                                             true,
    //                                             4_fps,
    //                                             4_fps_sq));
    // m_Modes["[4] blue source -> source far"].push_back(new StationaryVisionCommand(0.5_s));
    // m_Modes["[4] blue source -> source far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    // m_Modes["[4] blue source -> source far"].push_back(new WaitCommand(0.15_s,false));
    // m_Modes["[4] blue source -> source far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));

    // end
    m_Modes["[3] blue source -> source far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_STOW_SETPOINT"),
                                                            CONSTANT("PIVOT_STOW_SETPOINT"),
                                                            false));

    /* [3] blue source -> mid far */
    // paths: blue-source-drop-start -> blue-source-get_far-1 -> blue-source_get-mid-source_from-shoot

    // pre-load
    m_Modes["[3] blue source -> mid far"].push_back(preloadCommand(7.82,17.68));
    m_Modes["[3] blue source -> mid far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_STOW_SETPOINT"),
                                                        CONSTANT("PIVOT_STOW_SETPOINT"),
                                                        false));
    // m_Modes["[3] blue source -> mid far"].push_back(new WaitCommand(0.3_s,false)); // wait so we dont destroy arm

    // piece 2
    m_Modes["[3] blue source -> mid far"].push_back(pathWithEvents("blue-source-drop-start",
                                                { { 2.15_s, new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"), // was 1.6
                                                                                CONSTANT("PIVOT_GROUND_SETPOINT"),
                                                                                false)},
                                                  { 0.01_s, new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, false) },
                                                  { 1.1_s, new UpdateArmCommand(CONSTANT("WRIST_STOW_SETPOINT"), // was 1
                                                                                CONSTANT("PIVOT_STOW_SETPOINT"),
                                                                                false)},
                                                  { 2.2_s, new UpdateArmCommand(17.68,  // was 1.5
                                                                                CONSTANT("PIVOT_LAUNCH_SETPOINT"),
                                                                                false,
                                                                                true) },
                                                  { 0.01_s, new UpdateShooterStateCommand(Shooter::ShooterState::SPIN_UP, false) }},
                                                true,
                                                12_fps,
                                                12_fps_sq));
    m_Modes["[3] blue source -> mid far"].push_back(new StationaryVisionCommand(0.5_s));
    m_Modes["[3] blue source -> mid far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    m_Modes["[3] blue source -> mid far"].push_back(new WaitCommand(0.15_s,false));
    m_Modes["[3] blue source -> mid far"].push_back(new UpdateShooterStateCommand(Shooter::ShooterState::IDLE, false));
    m_Modes["[3] blue source -> mid far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));

    // piece 3
    m_Modes["[3] blue source -> mid far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
                                                        CONSTANT("PIVOT_GROUND_SETPOINT"),
                                                        false));
    m_Modes["[3] blue source -> mid far"].push_back(pathWithEvents("blue-source-get_far-1",
                                                { { 0.01_s, new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, false) },
                                                  { 3.5_s, new UpdateArmCommand(17.68,
                                                                                CONSTANT("PIVOT_LAUNCH_SETPOINT"),
                                                                                false,
                                                                                true) },
                                                  { 0.01_s, new UpdateShooterStateCommand(Shooter::ShooterState::SPIN_UP, false) }},
                                                true,
                                                12_fps,
                                                8_fps_sq));
    m_Modes["[3] blue source -> mid far"].push_back(new StationaryVisionCommand(0.5_s));
    m_Modes["[3] blue source -> mid far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    m_Modes["[3] blue source -> mid far"].push_back(new WaitCommand(0.15_s,false));
    m_Modes["[3] blue source -> mid far"].push_back(new UpdateShooterStateCommand(Shooter::ShooterState::IDLE, false));
    m_Modes["[3] blue source -> mid far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));

    // piece 4
    // m_Modes["[3] blue source -> mid far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_GROUND_SETPOINT"),
    //                                                     CONSTANT("PIVOT_GROUND_SETPOINT"),
    //                                                     false));
    // m_Modes["[3] blue source -> mid far"].push_back(pathWithEvents("blue-source_get-mid-source_from-shoot",
    //                                             { { 0.01_s, new UpdateIntakeStateCommand(Shooter::IntakeState::DETECT_ACTIVE, false) },
    //                                               { 2.3_s, new UpdateArmCommand(17.68,
    //                                                                             CONSTANT("PIVOT_LAUNCH_SETPOINT"),
    //                                                                             false,
    //                                                                             true) }},
    //                                             true,
    //                                             4_fps,
    //                                             4_fps_sq));
    // m_Modes["[3] blue source -> mid far"].push_back(new StationaryVisionCommand(0.5_s));
    // m_Modes["[3] blue source -> mid far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::SHOOT, false));
    // m_Modes["[3] blue source -> mid far"].push_back(new WaitCommand(0.15_s,false));

    // end
    m_Modes["[3] blue source -> mid far"].push_back(new UpdateShooterStateCommand(Shooter::ShooterState::IDLE, false));
    m_Modes["[3] blue source -> mid far"].push_back(new UpdateIntakeStateCommand(Shooter::IntakeState::IDLE, false));
    m_Modes["[3] blue source -> mid far"].push_back(new UpdateArmCommand(CONSTANT("WRIST_STOW_SETPOINT"),
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
