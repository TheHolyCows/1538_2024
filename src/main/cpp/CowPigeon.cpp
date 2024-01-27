#include "CowPigeon.h"

CowPigeon *CowPigeon::s_Instance = nullptr;

CowPigeon *CowPigeon::GetInstance()
{
    if (s_Instance == nullptr)
    {
        s_Instance = new CowPigeon();
    }

    return s_Instance;
}

CowPigeon::CowPigeon()
{
    constexpr int PIGEON_ID = 24;

    m_Pigeon = new ctre::phoenix6::hardware::Pigeon2(PIGEON_ID, "cowdrive");

    m_SynchronizedSignals.Yaw = &m_Pigeon->GetYaw();
    m_SynchronizedSignals.Pitch = &m_Pigeon->GetPitch();
    m_SynchronizedSignals.Roll = &m_Pigeon->GetRoll();

    m_Inverted = false;
}

std::vector<ctre::phoenix6::BaseStatusSignal*> CowPigeon::GetSynchronizedSignals()
{
    std::vector<ctre::phoenix6::BaseStatusSignal*> signals = {
        m_SynchronizedSignals.Yaw,
        m_SynchronizedSignals.Pitch,
        m_SynchronizedSignals.Roll
    };

    return signals;
}

void CowPigeon::SetInverted(bool inverted)
{
    m_Inverted = inverted;
}

units::degree_t CowPigeon::GetYaw()
{
    return m_SynchronizedSignals.Yaw->GetValue()* (m_Inverted ? -1 : 1);
}

units::degree_t CowPigeon::GetPitch()
{
    return m_SynchronizedSignals.Pitch->GetValue()* (m_Inverted ? -1 : 1);
}

units::degree_t CowPigeon::GetRoll()
{
    return m_SynchronizedSignals.Roll->GetValue()* (m_Inverted ? -1 : 1);
}

double CowPigeon::GetYawDegrees()
{
    return GetYaw().value();
}

double CowPigeon::GetPitchDegrees()
{
    return GetPitch().value();
}

double CowPigeon::GetRollDegrees()
{
    return GetRoll().value();
}

void CowPigeon::SetYaw(units::degree_t angle)
{
    m_Pigeon->SetYaw(angle);
}

void CowPigeon::SetYaw(double angle)
{
    SetYaw(units::degree_t{ angle });
}
