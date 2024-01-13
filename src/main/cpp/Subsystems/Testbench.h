#pragma once

#include <iostream>
#include "../CowLib/CowMotorController.h"


class Testbench
{
public:

    Testbench (int motorID1, int motorID2, int motorID3, int motorID4, int motorID5);
    void Handle();
    void SetMotor1 (double percent);
    void SetMotor2 (double percent);
    void SetMotor3 (double percent);
    void SetMotor4 (double percent);
    void SetMotor5 (double percent);

private:

    CowLib::CowMotorController *m_Motor1;
    CowLib::CowMotorController *m_Motor2;
    CowLib::CowMotorController *m_Motor3;
    CowLib::CowMotorController *m_Motor4;
    CowLib::CowMotorController *m_Motor5;
    CowMotor::PercentOutput m_Motor1ControlRequest{ 0 };
    CowMotor::PercentOutput m_Motor2ControlRequest{ 0 };
    CowMotor::PercentOutput m_Motor3ControlRequest{ 0 };
    CowMotor::PercentOutput m_Motor4ControlRequest{ 0 };
    CowMotor::PercentOutput m_Motor5ControlRequest{ 0 };

};
