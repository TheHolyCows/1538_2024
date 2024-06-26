//==================================================
// Copyright (C) 2022 Team 1538 / The Holy Cows
//==================================================

#ifndef __AUTO_MODES_H__
#define __AUTO_MODES_H__

#include "Commands/HoldPositionCommand.h"
#include "Commands/LambdaCommand.h"
#include "Commands/PathplannerSwerveCommand.h"
#include "Commands/PathplannerVisionCommand.h"
#include "Commands/RaceCommand.h"
#include "Commands/SeriesCommand.h"
#include "Commands/WaitCommand.h"
#include "Commands/ParallelCommand.h"
#include "Commands/StationaryVisionCommand.h"
#include "Commands/UpdateArmCommand.h"
#include "Commands/UpdateIntakeStateCommand.h"
#include "Commands/UpdateShooterStateCommand.h"
#include "Commands/UpdateShooterSpeed.h"

#include "../Subsystems/Shooter.h"
#include "../Subsystems/Wrist.h"
#include "../Subsystems/Pivot.h"

#include <deque>
#include <frc/Errors.h>
#include <map>
#include <string>

class AutoModes
{
private:
    AutoModes();
    ~AutoModes();
    static AutoModes *s_Instance;

    std::map<std::string, std::deque<RobotCommand *>> m_Modes;
    std::map<std::string, std::deque<RobotCommand *>>::iterator m_Iterator;

public:
    static AutoModes *GetInstance();

    std::deque<RobotCommand *> GetCommandList();

    std::string GetName();

    void NextMode();
};

#endif /* __AUTO_MODES_H__ */
