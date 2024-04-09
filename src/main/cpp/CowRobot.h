//==================================================
// Copyright (C) 2018 Team 1538 / The Holy Cows
//==================================================

#ifndef __COW_ROBOT_H__
#define __COW_ROBOT_H__

#include "Controllers/GenericController.h"
#include "CowConstants.h"
#include "CowLib/CowAlphaNum.h"
#include "CowLib/CowLogger.h"
#include "CowLib/CowPID.h"
#include "CowLib/CowTimer.h"
#include "CowLib/Utility.h"
#include "CowLib/CowInterp.h"
#include "CowPigeon.h"
#include "Drivetrain/SwerveDrive.h"
#include "Drivetrain/SwerveDriveController.h"
#include "frc/controller/PIDController.h"
#include "Vision.h"

#include "LoadManager.h"
#include "Subsystems/Pivot.h"
#include "Subsystems/Shooter.h"
#include "Subsystems/Elevator.h"
#include "Subsystems/Wrist.h"
#include "Subsystems/Fan.h"
#include <frc/BuiltInAccelerometer.h>
#include <frc/filter/LinearFilter.h>
#include <frc/PowerDistribution.h>
#include <math.h>
#include <vector>

class CowRobot
{
public:
    LoadManager* m_LoadManager;

    // Drive Motors
    SwerveDrive *m_Drivetrain;
    Pivot *m_Pivot;
    Shooter *m_Shooter;
    Elevator *m_Elevator;
    Wrist *m_Wrist;
    Vision *m_Vision;
    Fan *m_Fan;
    CowLib::interpolating_map<double, double> m_PivotRangeMap;
    CowLib::interpolating_map<double, double> m_ShooterRangeMap;

    std::optional<frc::DriverStation::Alliance> m_Alliance = std::nullopt;

    double m_BiasForAuto = 0.0;

private:
    int m_DSUpdateCount;

    GenericController *m_Controller = nullptr;

    SwerveDriveController *m_DriveController;

    // gyro and accelerometers
    CowPigeon *m_Gyro;
    frc::BuiltInAccelerometer *m_Accelerometer;
    frc::LinearFilter<double> m_ZFilter = frc::LinearFilter<double>::MovingAverage(12);
    double m_PrevZ;

    // display on rio removed
    CowLib::CowAlphaNum *m_LEDDisplay;

    std::vector<ctre::phoenix6::BaseStatusSignal*> m_CowDriveSignals;
    std::vector<ctre::phoenix6::BaseStatusSignal*> m_CowBusSignals;

    double m_MatchTime;
    double m_StartTime;

public:
    CowRobot();
    void Reset();
    void SetController(GenericController *controller);
    void PrintToDS();

    void StartTime();

    CowLib::CowAlphaNum *GetDisplay() { return m_LEDDisplay; }

    CowPigeon *GetGyro() { return CowPigeon::GetInstance(); }

    SwerveDrive *GetDrivetrain() { return m_Drivetrain; }

    SwerveDriveController *GetDriveController() { return m_DriveController; }

    void SampleSensors();

    void Handle();

    void DoNothing(void);

    void ClimbSM(void);
};

#endif