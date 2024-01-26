//==================================================
// Copyright (C) 2024 Team 1538 / The Holy Cows
// Elevator.h
// author: Kiran/Dustin/Jon
// created on: 2024-1-24
//==================================================

#include "Elevator.h"
#include "Shooter.h"

class Arm
{
public:
Arm(const int elevatorMotorID, 
    const int pivotMotorID, 
    const int shooterID1, 
    const int shooterID2, 
    const int intakeID1, 
    const int intakeID2, 
    const int wristID);

void ClimbSM(void);
void Handle();

private:

};