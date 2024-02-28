#include "CowBase.h"

CowBase::CowBase()
    : TimedRobot(10_ms),  // set robot duty cycle
      m_ControlBoard(new CowControlBoard()),
      m_OpController(new OperatorController(m_ControlBoard)),
      m_AutoController(new AutoModeController()),
      m_Constants(CowConstants::GetInstance())
{
    CowConstants::GetInstance()->RestoreData();
    m_Bot = new CowRobot();

    // m_Display = new CowDisplay(m_Bot); - removed from bot

    // init logger
    CowLib::CowLogger::GetInstance();

    // init gyro
    CowPigeon::GetInstance();

    // GetWatchdog().SetEnabled(false);
    printf("Done constructing CowBase!\n");
}

CowBase::~CowBase()
{
    delete m_ControlBoard;
    delete m_OpController;
    delete m_AutoController;
    // delete m_Display;
}

void CowBase::RobotInit()
{
    m_Bot->Reset();

    // Construct the auto modes class to load swerve trajectories
    AutoModes::GetInstance();

    CowPigeon::GetInstance()->SetYaw(0);
}

void CowBase::DisabledInit()
{
    CowConstants::GetInstance()->RestoreData();
    printf("DISABLED INIT -------------------\n");

    m_Bot->GetDriveController()->ResetHeadingLock();

    m_Bot->GetDrivetrain()->SetBrakeMode(true);
}

void CowBase::AutonomousInit()
{
    m_Bot->GetDrivetrain()->ResetEncoders();

    m_Bot->GetDrivetrain()->SetBrakeMode(true);

    m_AutoController->SetCommandList(AutoModes::GetInstance()->GetCommandList());
    std::cout << "Done setting command list" << std::endl;

    AutoModes::GetInstance()->NextMode();

    m_Bot->SetController(m_AutoController);
    m_Bot->Reset();

    CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "start auto mode");
    m_AutoController->Start(m_Bot);
}

void CowBase::TeleopInit()
{
    m_Bot->GetDrivetrain()->SetBrakeMode(false);

    m_Bot->StartTime();
    // m_Bot->GetGyro()->FinalizeCalibration();
    std::cout << "setting controller " << m_OpController << std::endl;
    m_Bot->SetController(m_OpController);
    std::cout << "controller set successfully" << std::endl;
    // m_Bot->GetArm()->SetBrakeMode(); TODO: add back in
}

void CowBase::DisabledPeriodic()
{
    // log motor info
    CowLib::CowLogger::GetInstance()->Handle();

    // m_Bot->GyroHandleCalibration();

    // if (m_Display)
    // {
    //     m_Display->DisplayPeriodic();
    // }

    if (m_ControlBoard->GetConstantsResetButton())
    {
        printf("RESETTING CONSTANTS\n");
        CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_OFF, "RESETTING CONSTANTS");
        m_Constants->RestoreData();
        m_Bot->Reset();
    }

    // if (m_ControlBoard->GetAutoSelectButton())
    // {
    //     /*
    //      * POSITION FIRST_OWNERSHIP SECOND_OWNERSHIP DRIVE
    //      * iterates over AutoModes
    //      */
    //     AutoModes::GetInstance()->NextMode();
    //     CowLib::CowLogger::LogAutoMode(m_Alliance, AutoModes::GetInstance()->GetName().c_str());
    //     printf("%s\n", AutoModes::GetInstance()->GetName().c_str());
    // }

    // if (m_Bot)
    // {
    //     // TODO: add this back in
    //     // m_Bot->GetArm()->DisabledCalibration();
    // }

    if (m_DisabledCount++ % 50 == 0) // update every .5 seconds
    {
        m_Alliance = frc::DriverStation::Alliance::kRed;
        CowLib::CowLogger::LogAutoMode(m_Alliance, AutoModes::GetInstance()->GetName().c_str());
        m_DisabledCount = 1;

        if (m_ControlBoard->GetOperatorButton(2)) // climb down
        {
            m_Bot->m_Pivot->BrakeMode(false);
            m_Bot->m_Wrist->BrakeMode(false);
            m_Bot->m_Elevator->BrakeMode(false);
        }
        else
        {
            m_Bot->m_Pivot->BrakeMode(true);
            m_Bot->m_Wrist->BrakeMode(true);
            m_Bot->m_Elevator->BrakeMode(true);
        }
    }
    
    // set wrist and pivot to current locations
    m_Bot->m_Pivot->SetAngle(m_Bot->m_Pivot->GetAngle());
    m_Bot->m_Wrist->SetAngle(m_Bot->m_Wrist->GetAngle(),true);
}

void CowBase::AutonomousPeriodic()
{
    m_Bot->Handle();
}

void CowBase::TeleopPeriodic()
{
    m_Bot->Handle();
}

int main()
{
    return frc::StartRobot<CowBase>();
}
