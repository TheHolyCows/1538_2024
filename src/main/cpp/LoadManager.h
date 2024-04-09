#pragma once

#include <frc/PowerDistribution.h>
#include <units/energy.h>
#include <units/power.h>

class LoadManager
{
public:
    LoadManager();

    units::joule_t GetEnergyConsumed();
    units::watt_t GetInstantaneousLoad();

    void Handle();
private:
    frc::PowerDistribution m_PDH;
    units::joule_t m_EnergyConsumed;
    units::watt_t m_InstantaneousLoad;
};