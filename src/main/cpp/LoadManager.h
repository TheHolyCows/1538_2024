#pragma once

#include <frc/PowerDistribution.h>
#include <units/energy.h>
#include <units/power.h>
#include <units/current.h>


class LoadManager
{
public:
    LoadManager();

    units::joule_t GetEnergyConsumed();
    units::watt_t GetInstantaneousLoad();

    units::ampere_t GetSwerveDriveBudget();

    void Handle();
private:
    frc::PowerDistribution m_PDH;
    units::joule_t m_EnergyConsumed;
    units::watt_t m_InstantaneousLoad;
};