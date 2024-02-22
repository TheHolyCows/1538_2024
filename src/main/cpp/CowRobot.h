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
#include "CowPigeon.h"
#include "Drivetrain/SwerveDrive.h"
#include "Drivetrain/SwerveDriveController.h"
#include "frc/controller/PIDController.h"

// #include "Subsystems/ArmState.h" // delete before 2024
// #include "Subsystems/Vision.h"
#include "Subsystems/Pivot.h"
#include "Subsystems/Shooter.h"
// #include "Subsystems/Elevator.h"
#include <frc/BuiltInAccelerometer.h>
#include <frc/filter/LinearFilter.h>
#include <frc/PowerDistribution.h>
#include <math.h>
#include <vector>

class CowRobot
{
public:
    // Drive Motors
    SwerveDrive *m_Drivetrain;
    Pivot *m_Pivot;
    Shooter *m_Shooter;
    // Elevator *m_Elevator;

private:
    std::vector<ctre::phoenix6::BaseStatusSignal*> GetSynchronizedSignals();

    int m_DSUpdateCount;

    GenericController *m_Controller = nullptr;

    SwerveDriveController *m_DriveController;

    // gyro and accelerometers
    CowPigeon *m_Gyro;
    frc::BuiltInAccelerometer *m_Accelerometer;
    frc::LinearFilter<double> m_ZFilter = frc::LinearFilter<double>::MovingAverage(12);
    double m_PrevZ;

    // PDP
    frc::PowerDistribution *m_PowerDistributionPanel;

    // display on rio removed
    CowLib::CowAlphaNum *m_LEDDisplay;

    double m_LeftDriveValue;
    double m_RightDriveValue;

    double m_PreviousDriveError;

    double m_MatchTime;
    double m_StartTime;

public:
    CowRobot();
    void Reset();
    void SetController(GenericController *controller);
    void PrintToDS();

    void StartTime();

    CowLib::CowAlphaNum *GetDisplay() { return m_LEDDisplay; }

    frc::PowerDistribution *GetPowerDistributionPanel() { return m_PowerDistributionPanel; }

    CowPigeon *GetGyro() { return CowPigeon::GetInstance(); }

    SwerveDrive *GetDrivetrain() { return m_Drivetrain; }

    SwerveDriveController *GetDriveController() { return m_DriveController; }

    void Handle();

    void DoNothing(void);

    void ClimbSM(void);
};



#endif