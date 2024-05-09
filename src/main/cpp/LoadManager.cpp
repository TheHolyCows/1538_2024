#include "LoadManager.h"

#include <units/current.h>
#include <units/voltage.h>
#include <units/power.h>
#include <units/energy.h>
#include <units/frequency.h>
#include <units/time.h>

#include "CowConstants.h"

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
    return m_InstantaneousLoad;
}

units::ampere_t LoadManager::GetSwerveDriveBudget()
{
    double limit = ((m_EnergyConsumed / 3600_s).value() * CONSTANT("LOAD_MANAGER_SWERVE_M")) + CONSTANT("LOAD_MANAGER_SWERVE_B");
    return units::ampere_t(std::clamp(limit, CONSTANT("LOAD_MANAGER_SWERVE_MIN"), CONSTANT("LOAD_MANAGER_SWERVE_MAX")));
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