//==================================================
// Copyright (C) 2024 Team 1538 / The Holy Cows
// Shooter.h
// author: Kiran/Dustin/Jon
// created on: 2024-1-16
//==================================================

#pragma once

#include "../CowLib/CowMotor/TalonFX.h"
#include "../Cowconstants.h"
#include "../Cowlib/CowLPF.h"
#include "../CowLib/Conversions.h"
#include "../CowLib/CowLogger.h"
#include "../CowLib/CowCANCoder.h"

class Fan
{
public:
    enum class FanState {
        IDLE,
        SPIN_UP,
        FULL
    };

    Fan(const int FanMotorID);

    void ResetConstants();

    FanState GetFanState();
    FanState UpdateFanState(FanState newstate);

    double GetFanVel();

    void FanOn();
    void FanOff();

    void Handle();

private:
    std::unique_ptr<CowMotor::TalonFX> m_FanMotor;

    FanState m_FanState;

    double m_FanVel;
};