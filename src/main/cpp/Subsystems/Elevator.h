//==================================================
// Copyright (C) 2024 Team 1538 / The Holy Cows
// Elevator.h
// author: Kiran/Dustin/Jon
// created on: 2024-1-16
//==================================================

#pragma once

#include "../CowConstants.h"
#include "../CowLib/Conversions.h"
#include "../CowLib/CowMotorController.h"

#include <memory>
#include <iostream>

class Elevator
{
public:
    enum PIDSet {
        EXTENDING,
        RETRACTING,
    };

    Elevator(const int MotorId);

    /**
     * sets variable in current position request to pos
    */
    void RequestPosition(double pos);

    double GetSetpoint();

    bool AtTarget();

    /**
     * returns the current encoder read from the motor
     * not currently converted
    */
    double GetPosition();

    /**
     * update PID of pivot based on currrent arm extension
    */
    void UpdatePID(double);

    void UsePIDSet(PIDSet set);

    void ResetConstants();

    void Handle();

    void BrakeMode(bool brakeMode);

private:
    std::shared_ptr<CowLib::CowMotorController> m_ElevatorMotor;
    CowMotor::MotionMagicPercentOutput m_MotorRequest = { 0 };

    PIDSet m_PrevPIDSet = RETRACTING;

//     CowMotor::PercentOutput m_MotorRequest = {0};
};