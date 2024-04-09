#include "LoadManager.h"

#include <units/current.h>
#include <units/voltage.h>
#include <units/power.h>
#include <units/energy.h>
#include <units/frequency.h>

LoadManager::LoadManager()
    : m_PDH(1, frc::PowerDistribution::ModuleType::kRev),
      m_EnergyConsumed(0_J),
      m_InstantaneousLoad(0_W)
{

}

units::joule_t LoadManager::GetEnergyConsumed()
{
    return m_EnergyConsumed;
}

units::watt_t LoadManager::GetInstantaneousLoad()
{

}

void LoadManager::Handle()
{
    units::volt_t voltage = units::volt_t(m_PDH.GetVoltage());
    units::ampere_t current = units::ampere_t(m_PDH.GetTotalCurrent());
    units::watt_t watts = voltage * current;
    units::joule_t joules = watts * (1.0 / 100.0_Hz);

    m_EnergyConsumed += joules;
    m_InstantaneousLoad = watts;
}