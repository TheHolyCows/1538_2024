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
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/velocity.h>

#include <memory>
#include <iostream>

class Elevator
{
public:
    enum PIDSet {
        EXTENDING,
        RETRACTING,
    };

    enum CLIMB_STATE {
        ST_EXTEND,
        ST_RETRACT,
        ST_DEFAULT,
    };

    Elevator(const int elevatorMotorID, const int pivotMotorID);

    CLIMB_STATE m_ClimberState;

    /**
     * sets variable in current position request to pos
    */
    void RequestElevatorPosition(double pos);

    double GetElevatorSetpoint();

    bool ElevatorAtTarget();

    /**
     * returns the current encoder read from the motor
     * not currently converted
    */
    double GetElevatorPosition();

    void RequestPivotAngle(double angle);

    double GetPivotSetpoint();

    bool PivotAtTarget();

    /**
     * returns the current angle read from the motor
     * converted to degrees
    */
    double GetPivotAngle();

    /**
     * update PID of pivot based on currrent arm extension
    */
    void UpdatePID(double);

    void UsePIDSet(PIDSet set);

    void ResetConstants();

    void Handle();

    void PivotBrakeMode(bool brakeMode);

private:
    std::shared_ptr<CowLib::CowMotorController> m_ElevatorMotor;
    std::shared_ptr<CowLib::CowMotorController> m_PivotMotor;
    CowMotor::MotionMagicPercentOutput m_ElevatorMotorRequest = { 0 };
    CowMotor::MotionMagicPercentOutput m_PivotMotorRequest = { 0 };
    
    double m_TargetAngle;
    int m_TickCount;

    PIDSet m_PrevPIDSet = RETRACTING;
};